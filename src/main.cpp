#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <vector>
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "hal.h"
#include "web_ui.h"

// ================== I2C ==================
#define I2C_SDA 21
#define I2C_SCL 22

// ================== PWM INPUT PINS (ESP32) ==================
static const int PWM_PINS[8] = {16,17,18,19,23,4,5,34};
static const rmt_channel_t RMT_CH[8] = {
  RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3,
  RMT_CHANNEL_4, RMT_CHANNEL_5, RMT_CHANNEL_6, RMT_CHANNEL_7
};

static RingbufHandle_t rb[8] = {nullptr};
static volatile uint16_t pwmWidth[8] = {0};
static volatile uint32_t pwmStampMs[8] = {0}; // РєРѕРіРґР° РѕР±РЅРѕРІРёР»Рё

// ================== UI ==================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);
Preferences prefs;

// ================== RULES ==================
enum ActType { BTS, PCA_OUT, REVERSER };
enum Mode    { RELAY, POLARITY_RELAY, ENGINE, BTS_Mode };

struct Rule {
  enum CondKind { ANY, BELOW, ABOVE, BETWEEN } kind = ANY;
  enum BtsKind { BTS_FIXED, BTS_RANGE } btsKind = BTS_FIXED;
  uint16_t aUs = 1500, bUs = 2000;
  uint16_t outAUs = 1000, outBUs = 2000;
  int16_t btsPct = 0;
  int16_t btsMinPct = -100, btsMaxPct = 100;
  ActType type = PCA_OUT;
  uint8_t targetIndex = 0;   // 0..26
  Mode mode = RELAY;
};

static std::vector<Rule> rules[8];

// target index mapping:
// 0..15 -> PCA9685 ch (ENGINE PWM)
// 16..19 -> BTS 0..3
// 20..23 -> Relay 0..3
// 24..26 -> Polarity relay 0..2
static inline void applyOutputByTarget(
    uint8_t tgt,
    Mode mode,
    float value01,
    float valueSigned
) {
    if (tgt <= 15) {
        setEnginePwm(tgt, value01);
    }
    else if (tgt <= 19) {
        setBts(tgt - 16, valueSigned);
    }
    else if (tgt <= 23) {
        setRelay(tgt - 20, value01 > 0.5f);
    }
    else if (tgt <= 26) {
        bool on = fabs(valueSigned) > 0.05f;
        bool pol = valueSigned >= 0;
        setPolarityRelay(tgt - 24, on, pol);
    }
}

// ================== OUTPUT STATE ==================
struct OutputState {
  bool active = false;
  float v01 = 0.0f;      // 0..1
  float vs  = 0.0f;      // -1..+1
};
static volatile OutputState outState[32];
static const bool SERIAL_DEBUG_ENABLED = true;
static const uint32_t SERIAL_DEBUG_POLL_MS = 50;

// ================== MENU ==================
enum MenuState { MAIN, STATUSS, CHSEL, RULES_LIST, EDIT_RULE, SAVED };
static volatile MenuState menu = MAIN;

static int mainIdx=0, chIdx=0, ruleIdx=0, fieldIdx=0;
static int selCh=0;
static Rule* edit = nullptr;

// ================== helpers ==================
static inline int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline const char* targetName(uint8_t idx){
  if(idx <= 15) return "PCA";
  if(idx <= 19) return "BTS";
  if(idx <= 23) return "Relay";
  if(idx <= 26) return "Polarity";
  return "?";
}
static inline int targetLocalIndex(uint8_t idx){
  if(idx <= 15) return idx;
  if(idx <= 19) return idx - 16;
  if(idx <= 23) return idx - 20;
  if(idx <= 26) return idx - 24;
  return -1;
}

static const char* actName(ActType t){
  switch(t){ case BTS:return "BTS"; case PCA_OUT:return "PCA"; case REVERSER:return "REV"; }
  return "?";
}
static const char* condName(Rule::CondKind c){
  switch(c){ case Rule::ANY:return "ANY"; case Rule::BELOW:return "Меньше"; case Rule::ABOVE:return "Больше"; case Rule::BETWEEN:return "Между"; }
  return "?";
}
static const char* modeName(Mode m){
  switch(m){ case RELAY:return "Реле"; case POLARITY_RELAY:return "Полярность"; case ENGINE:return "ШИМ вывод"; case BTS_Mode:return "BTS"; }
  return "?";
}
static const char* btsKindName(Rule::BtsKind k){
  switch(k){ case Rule::BTS_FIXED:return "Фикс"; case Rule::BTS_RANGE:return "Диапазон"; }
  return "?";
}

static inline bool condOk(const Rule& r, uint16_t pwm){
  switch(r.kind){
    case Rule::ANY: return true;
    case Rule::BELOW: return pwm < r.aUs;
    case Rule::ABOVE: return pwm > r.aUs;
    case Rule::BETWEEN: return pwm >= r.aUs && pwm <= r.bUs;
  }
  return false;
}

// pwm -> values
static inline float pwm_to_01(uint16_t us){
  float v = (us - 1000.0f) / 1000.0f;
  return constrain(v, 0.0f, 1.0f);
}
static inline float pwm_to_signed(uint16_t us){
  float v = (us - 1500.0f) / 500.0f;
  return constrain(v, -1.0f, 1.0f);
}
static inline float map_range_clamped(float x, float inA, float inB, float outA, float outB){
  if (fabsf(inB - inA) < 0.001f) return outA;
  float t = (x - inA) / (inB - inA);
  t = constrain(t, 0.0f, 1.0f);
  return outA + (outB - outA) * t;
}
static inline float compute_bts_value(uint16_t us, const Rule& r){
  switch (r.kind) {
    case Rule::ABOVE:
      return constrain(map_range_clamped((float)us, (float)r.aUs, 2000.0f, (float)r.btsMinPct, (float)r.btsMaxPct) / 100.0f, -1.0f, 1.0f);
    case Rule::BELOW:
      return constrain(map_range_clamped((float)us, (float)r.aUs, 1000.0f, (float)r.btsMinPct, (float)r.btsMaxPct) / 100.0f, -1.0f, 1.0f);
    case Rule::BETWEEN:
      return constrain(map_range_clamped((float)us, (float)r.aUs, (float)r.bUs, (float)r.btsMinPct, (float)r.btsMaxPct) / 100.0f, -1.0f, 1.0f);
    case Rule::ANY:
    default:
      return constrain(map_range_clamped((float)us, 1000.0f, 2000.0f, (float)r.btsMinPct, (float)r.btsMaxPct) / 100.0f, -1.0f, 1.0f);
  }

  // Fixed BTS mode is temporarily disabled.
  // return constrain((float)r.btsPct / 100.0f, -1.0f, 1.0f);
}

// ================== NVS SAVE/LOAD ==================
void saveRules() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("rules");
  for(int ch=0; ch<8; ch++){
    JsonArray a = arr.createNestedArray();
    for(auto &r: rules[ch]){
      JsonObject o = a.createNestedObject();
      o["type"]=(int)r.type;
      o["cond"]=(int)r.kind;
      o["a"]=r.aUs;
      o["b"]=r.bUs;
      o["outA"]=r.outAUs;
      o["outB"]=r.outBUs;
      o["btsKind"]=(int)r.btsKind;
      o["bts"]=r.btsPct;
      o["btsMin"]=r.btsMinPct;
      o["btsMax"]=r.btsMaxPct;
      o["tgt"]=r.targetIndex;
      o["mode"]=(int)r.mode;
    }
  }
  String s; serializeJson(doc,s);
  prefs.putString("cfg", s);
  Serial.printf("[NVS] saved len=%d\n", (int)s.length());
}

void loadRules() {
  if(!prefs.isKey("cfg")){
    Serial.println("[NVS] cfg missing -> defaults");
    return;
  }
  String s = prefs.getString("cfg");
  DynamicJsonDocument doc(4096);
  if(deserializeJson(doc,s)){
    Serial.println("[NVS] json error");
    return;
  }
  JsonArray arr = doc["rules"];
  for(int ch=0; ch<8 && ch<(int)arr.size(); ch++){
    rules[ch].clear();
    for(JsonObject o: arr[ch].as<JsonArray>()){
      Rule r;
      r.type = (ActType)(int)o["type"];
      r.kind = (Rule::CondKind)(int)o["cond"];
      r.btsKind = o["btsKind"].isNull() ? Rule::BTS_FIXED : (Rule::BtsKind)(int)o["btsKind"];
      r.aUs  = (uint16_t)(int)o["a"];
      r.bUs  = (uint16_t)(int)o["b"];
      r.outAUs = o["outA"].isNull() ? 1000 : (uint16_t)(int)o["outA"];
      r.outBUs = o["outB"].isNull() ? 2000 : (uint16_t)(int)o["outB"];
      r.btsPct = o["bts"].isNull() ? 0 : (int16_t)(int)o["bts"];
      r.btsMinPct = o["btsMin"].isNull() ? -100 : (int16_t)(int)o["btsMin"];
      r.btsMaxPct = o["btsMax"].isNull() ? 100 : (int16_t)(int)o["btsMax"];
      r.targetIndex = (uint8_t)(int)o["tgt"];
      r.mode = (Mode)(int)o["mode"];
      rules[ch].push_back(r);
    }
    Serial.printf("[NVS] ch%d rules=%d\n", ch, (int)rules[ch].size());
  }
}

// ================== RMT INIT ==================
static void initRmtOne(int idx){
  gpio_num_t pin = (gpio_num_t)PWM_PINS[idx];

  // input mode
  gpio_set_direction(pin, GPIO_MODE_INPUT);

  // 34..39: no internal pullups
  if (pin != GPIO_NUM_34) {
    gpio_pullup_en(pin);
    gpio_pulldown_dis(pin);
  }

  rmt_config_t cfg = {};
  cfg.rmt_mode = RMT_MODE_RX;
  cfg.channel  = RMT_CH[idx];
  cfg.gpio_num = pin;
  cfg.mem_block_num = 1;
  cfg.clk_div = 80; // 1us tick
  cfg.rx_config.filter_en = true;
  cfg.rx_config.filter_ticks_thresh = 10;
  cfg.rx_config.idle_threshold = 2500; // РєР»СЋС‡РµРІРѕРµ РґР»СЏ RC: С„РёРєСЃРёСЂСѓРµРј РёРјРїСѓР»СЊСЃС‹ ~1000-2000us

  ESP_ERROR_CHECK(rmt_config(&cfg));
  // ringbuffer size: РќР• РѕРіСЂРѕРјРЅС‹Р№, Р° РґРѕСЃС‚Р°С‚РѕС‡РЅРѕ + РјС‹ РґСЂРµРЅРёСЂСѓРµРј
  ESP_ERROR_CHECK(rmt_driver_install(cfg.channel, 2048, 0));
  ESP_ERROR_CHECK(rmt_get_ringbuf_handle(cfg.channel, &rb[idx]));
  ESP_ERROR_CHECK(rmt_rx_start(cfg.channel, true));
}

static void initPWM(){
  for(int i=0;i<8;i++) initRmtOne(i);
  Serial.println("[PWM] RMT ready (8ch)");
}

// ================== TASK: PWM ==================
// РџСЂРѕС„. СЂРµС€РµРЅРёРµ РѕС‚ BUFFER FULL: РЅР° РєР°Р¶РґРѕРј РєР°РЅР°Р»Рµ "РґСЂРµРЅРёСЂСѓРµРј" РѕС‡РµСЂРµРґСЊ РґРѕ РїСѓСЃС‚Рѕ
void pwmTask(void*){
  Serial.println("[PWM] pwmTask started");
  for(;;){
    for(int ch=0; ch<8; ch++){
      if(!rb[ch]) continue;

      size_t sz=0;
      rmt_item32_t* items=nullptr;

      // РґСЂРµРЅРёСЂСѓРµРј: РїРѕРєР° РµСЃС‚СЊ РґР°РЅРЅС‹Рµ вЂ” С‡РёС‚Р°РµРј
      while((items = (rmt_item32_t*)xRingbufferReceive(rb[ch], &sz, 0)) != nullptr){
        int count = sz / sizeof(rmt_item32_t);
        // Р±РµСЂС‘Рј РїРѕСЃР»РµРґРЅРёР№ РІР°Р»РёРґРЅС‹Р№ РёРјРїСѓР»СЊСЃ РёР· РїР°С‡РєРё
        for(int i=0;i<count;i++){
          uint16_t us=0;
          if(items[i].level0==1) us = items[i].duration0;
          else if(items[i].level1==1) us = items[i].duration1;

          if(us>=900 && us<=2100){
            pwmWidth[ch]=us;
            pwmStampMs[ch]=millis();
          }
        }
        vRingbufferReturnItem(rb[ch], items);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ================== TASK: RULE ==================
void logicTask(void*){
  Serial.println("[LOGIC] logicTask started");
  for(;;){
    uint32_t now = millis();
    OutputState next[32];

    for(int i=0;i<32;i++){
      next[i].active = false;
      next[i].v01 = 0.0f;
      next[i].vs = 0.0f;
    }

    for(int ch=0; ch<8; ch++){
      uint16_t us = pwmWidth[ch];
      // timeout -> no signal on this input channel
      if(now - pwmStampMs[ch] > 120) continue;
      if(us < 900 || us > 2100) continue;

      float v01 = pwm_to_01(us);
      float vs  = pwm_to_signed(us);

      for(auto &r : rules[ch]){
        uint8_t out = r.targetIndex;
        if(out >= 32 || !condOk(r, us)) continue;

        // Last matching rule wins for the same output.
        next[out].active = true;

        // Primary behavior by output target group.
        if(out <= 15){
          float mappedOutUs = map_range_clamped((float)us, (float)r.aUs, (float)r.bUs, (float)r.outAUs, (float)r.outBUs);
          next[out].v01 = constrain((mappedOutUs - 1000.0f) / 1000.0f, 0.0f, 1.0f);
          next[out].vs  = 0.0f;
        } else if(out <= 19){
          next[out].v01 = 1.0f;
          next[out].vs  = compute_bts_value(us, r);
        } else if(out <= 23){
          next[out].v01 = 1.0f;     // Relay ON when condition is true
          next[out].vs  = 0.0f;
        } else if(out <= 26){
          next[out].v01 = 1.0f;
          next[out].vs  = vs;       // Polarity by sign
        }

        // Optional mode override (kept for compatibility with saved rules).
        switch(r.mode){
          case RELAY:
            next[out].v01 = 1.0f;
            next[out].vs  = 0.0f;
            break;
          case POLARITY_RELAY:
            next[out].v01 = 1.0f;
            next[out].vs  = vs;
            break;
          case ENGINE:
            if(out <= 15){
              float mappedOutUs = map_range_clamped((float)us, (float)r.aUs, (float)r.bUs, (float)r.outAUs, (float)r.outBUs);
              next[out].v01 = constrain((mappedOutUs - 1000.0f) / 1000.0f, 0.0f, 1.0f);
            } else {
              next[out].v01 = v01;
            }
            next[out].vs  = 0.0f;
            break;
          case BTS_Mode:
            next[out].v01 = 1.0f;
            if(out <= 19){
              next[out].vs = compute_bts_value(us, r);
            } else next[out].vs = vs;
            break;
        }
      }
    }

    for(int i=0;i<32;i++){
      outState[i].active = next[i].active;
      outState[i].v01    = next[i].v01;
      outState[i].vs     = next[i].vs;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

String buildRulesJson() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("rules");
  for (int ch = 0; ch < 8; ch++) {
    JsonArray a = arr.createNestedArray();
    for (auto &r : rules[ch]) {
      JsonObject o = a.createNestedObject();
      o["type"] = (int)r.type;
      o["cond"] = (int)r.kind;
      o["a"] = r.aUs;
      o["b"] = r.bUs;
      o["outA"] = r.outAUs;
      o["outB"] = r.outBUs;
      o["btsKind"] = (int)r.btsKind;
      o["bts"] = r.btsPct;
      o["btsMin"] = r.btsMinPct;
      o["btsMax"] = r.btsMaxPct;
      o["tgt"] = r.targetIndex;
      o["mode"] = (int)r.mode;
    }
  }
  String s;
  serializeJsonPretty(doc, s);
  return s;
}

bool applyRulesJson(const String& s) {
  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, s)) return false;
  JsonArray arr = doc["rules"];
  if (arr.isNull()) return false;

  for (int ch = 0; ch < 8; ch++) {
    rules[ch].clear();
    if (ch >= (int)arr.size()) continue;

    JsonArray one = arr[ch].as<JsonArray>();
    for (JsonObject o : one) {
      Rule r;
      r.type = (ActType)clampi((int)o["type"], 0, 2);
      r.kind = (Rule::CondKind)clampi((int)o["cond"], 0, 3);
      r.btsKind = o["btsKind"].isNull() ? Rule::BTS_FIXED : (Rule::BtsKind)clampi((int)o["btsKind"], 0, 1);
      r.aUs = (uint16_t)clampi((int)o["a"], 900, 2100);
      r.bUs = (uint16_t)clampi((int)o["b"], 900, 2500);
      r.outAUs = o["outA"].isNull() ? 1000 : (uint16_t)clampi((int)o["outA"], 0, 4095);
      r.outBUs = o["outB"].isNull() ? 2000 : (uint16_t)clampi((int)o["outB"], 0, 4095);
      r.btsPct = o["bts"].isNull() ? 0 : (int16_t)clampi((int)o["bts"], -100, 100);
      r.btsMinPct = o["btsMin"].isNull() ? -100 : (int16_t)clampi((int)o["btsMin"], -100, 100);
      r.btsMaxPct = o["btsMax"].isNull() ? 100 : (int16_t)clampi((int)o["btsMax"], -100, 100);
      r.targetIndex = (uint8_t)clampi((int)o["tgt"], 0, 26);
      r.mode = (Mode)clampi((int)o["mode"], 0, 3);
      rules[ch].push_back(r);
    }
  }
  return true;
}

String buildStatusJson() {
  DynamicJsonDocument doc(8192);
  JsonArray in = doc.createNestedArray("inputs");
  for (int i = 0; i < 8; i++) {
    JsonObject o = in.createNestedObject();
    o["ch"] = i;
    o["us"] = pwmWidth[i];
    o["ageMs"] = (uint32_t)(millis() - pwmStampMs[i]);
  }

  JsonArray out = doc.createNestedArray("outputs");
  for (int i = 0; i < 27; i++) {
    JsonObject o = out.createNestedObject();
    o["idx"] = i;
    o["active"] = outState[i].active;
    o["v01"] = outState[i].v01;
    o["vs"] = outState[i].vs;
    if (i <= 15) {
      o["pcaUs"] = (int)lroundf(1000.0f + outState[i].v01 * 1000.0f);
      o["pca12"] = (int)lroundf(outState[i].v01 * 4095.0f);
    }
  }

  String s;
  serializeJsonPretty(doc, s);
  return s;
}

// ================== TASK: OUTPUT ==================
void outputTask(void *p)
{
    Serial.println("[OUT] outputTask started");

    for (;;)
    {
        for (int i = 0; i < 32; i++)
        {
            if (!outState[i].active)
            {
                if (i <= 15) setEnginePwm(i, 0.0f);
                else if (i <= 19) setBts(i - 16, 0.0f);
                else if (i <= 23) setRelay(i - 20, false);
                else if (i <= 26) setPolarityRelay(i - 24, false, false);
                continue;
            }

            float v01 = outState[i].v01;
            float vs  = outState[i].vs;

            if (i <= 15)
            {
                setEnginePwm(i, v01);
            }
            else if (i <= 19)
            {
                setBts(i - 16, vs);
            }
            else if (i <= 23)
            {
                setRelay(i - 20, v01 > 0.5f);
            }
            else if (i <= 26)
            {
                bool on = fabs(vs) > 0.1f;
                bool forward = (vs > 0);
                setPolarityRelay(i - 24, on, forward);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void debugTask(void *p)
{
    if (!SERIAL_DEBUG_ENABLED)
    {
        vTaskDelete(nullptr);
        return;
    }

    Serial.println("[DBG] debugTask started");

    uint16_t prevPwm[8] = {0};
    bool prevPwmValid[8] = {0};
    OutputState prevOut[27];
    bool first = true;

    for (int i = 0; i < 27; i++)
    {
        prevOut[i].active = false;
        prevOut[i].v01 = 0.0f;
        prevOut[i].vs = 0.0f;
    }

    for (;;)
    {
        bool inputsChanged = false;
        bool outputsChanged = false;

        for (int ch = 0; ch < 8; ch++)
        {
            uint32_t age = millis() - pwmStampMs[ch];
            bool valid = (age <= 120 && pwmWidth[ch] >= 900 && pwmWidth[ch] <= 2100);
            if (first || valid != prevPwmValid[ch] || (valid && pwmWidth[ch] != prevPwm[ch]))
                inputsChanged = true;
        }

        for (int i = 0; i < 27; i++)
        {
            if (first ||
                outState[i].active != prevOut[i].active ||
                fabsf(outState[i].v01 - prevOut[i].v01) > 0.01f ||
                fabsf(outState[i].vs - prevOut[i].vs) > 0.01f)
            {
                outputsChanged = true;
                break;
            }
        }

        if (inputsChanged || outputsChanged)
        {
            Serial.println("[DBG] ------------------------------");

            if (inputsChanged)
            {
                Serial.print("[DBG] IN  ");
                for (int ch = 0; ch < 8; ch++)
                {
                    uint32_t age = millis() - pwmStampMs[ch];
                    bool valid = (age <= 120 && pwmWidth[ch] >= 900 && pwmWidth[ch] <= 2100);
                    if (valid) Serial.printf("CH%d=%u ", ch + 1, pwmWidth[ch]);
                    else Serial.printf("CH%d=--- ", ch + 1);
                }
                Serial.println();
            }

            if (outputsChanged)
            {
                bool anyActive = false;
                for (int i = 0; i < 27; i++)
                {
                    if (!outState[i].active) continue;
                    anyActive = true;

                    if (i <= 15)
                    {
                        int pcaUs = (int)lroundf(1000.0f + outState[i].v01 * 1000.0f);
                        int pca12 = (int)lroundf(outState[i].v01 * 4095.0f);
                        Serial.printf("[DBG] OUT %s%d active=1 v01=%.2f us=%d pwm12=%d\n",
                                      targetName(i), targetLocalIndex(i), outState[i].v01, pcaUs, pca12);
                    }
                    else if (i <= 19)
                    {
                        Serial.printf("[DBG] OUT %s%d active=1 vs=%.2f\n",
                                      targetName(i), targetLocalIndex(i), outState[i].vs);
                    }
                    else if (i <= 23)
                    {
                        Serial.printf("[DBG] OUT %s%d active=1 state=%s\n",
                                      targetName(i), targetLocalIndex(i), outState[i].v01 > 0.5f ? "ON" : "OFF");
                    }
                    else
                    {
                        Serial.printf("[DBG] OUT %s%d active=1 on=%s dir=%s vs=%.2f\n",
                                      targetName(i), targetLocalIndex(i),
                                      fabs(outState[i].vs) > 0.1f ? "YES" : "NO",
                                      outState[i].vs >= 0 ? "FWD" : "REV",
                                      outState[i].vs);
                    }
                }

                if (!anyActive)
                {
                    Serial.println("[DBG] OUT none active");
                }
            }
        }

        for (int ch = 0; ch < 8; ch++)
        {
            uint32_t age = millis() - pwmStampMs[ch];
            prevPwmValid[ch] = (age <= 120 && pwmWidth[ch] >= 900 && pwmWidth[ch] <= 2100);
            prevPwm[ch] = pwmWidth[ch];
        }

        for (int i = 0; i < 27; i++)
        {
            prevOut[i].active = outState[i].active;
            prevOut[i].v01 = outState[i].v01;
            prevOut[i].vs = outState[i].vs;
        }

        first = false;
        vTaskDelay(pdMS_TO_TICKS(SERIAL_DEBUG_POLL_MS));
    }
}

// ================== SIMPLE MCP ENCODER (polling) ==================
 
static inline bool encBtnClick(){ return halReadEncBtn(); }


// ================== DRAW ==================
void drawMain(){
  u8g2.clearBuffer();
u8g2.setFont(u8g2_font_6x12_t_cyrillic);
  const char* m[]={"Статус","Ред. правила","Сохр","Загр"};
  for(int i=0;i<4;i++){
    int y=20+i*12;
    if(i==mainIdx){ u8g2.drawBox(0,y-10,128,12); u8g2.setDrawColor(0); }


    u8g2.drawUTF8(4,y,m[i]);
    if(i==mainIdx) u8g2.setDrawColor(1);
  }
  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

// Р±РѕР»РµРµ Р°РєРєСѓСЂР°С‚РЅС‹Р№ Status: С€РєР°Р»Р° + С‡РёСЃР»Рѕ СЃРІРµСЂС…Сѓ
void drawStatus(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_t_cyrillic);

  const int barW=14, gap=2;
  const int topY=10;
  const int barBase=62;
  const int barMaxH=40;

  for(int i=0;i<8;i++){
    int x=i*(barW+gap);

    uint16_t us = pwmWidth[i];
    int h = 0;
    if(us>=900 && us<=2100){
      h = (int)lroundf((us-1000)*barMaxH/1000.0f);
      h = clampi(h,0,barMaxH);
    }

    // С‡РёСЃР»Рѕ СЃРІРµСЂС…Сѓ
    char t[8];
    if(us>=900 && us<=2100) snprintf(t,sizeof(t),"%u",us);
    else snprintf(t,sizeof(t),"---");
    u8g2.drawUTF8(x, topY, t);

    // Р±Р°СЂ СЃРЅРёР·Сѓ
    u8g2.drawFrame(x, barBase-barMaxH, barW, barMaxH);
    u8g2.drawBox(x+1, barBase-h, barW-2, h);
  }

  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

void drawCh(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);
  for(int i=0;i<8;i++){
    char b[24];
    snprintf(b,sizeof(b),"Канал %d (%d)", i+1, (int)rules[i].size());
    int y=20+i*12;
    if(i==chIdx){ u8g2.drawBox(0,y-10,128,12); u8g2.setDrawColor(0); }
    u8g2.drawUTF8(4,y,b);
    if(i==chIdx) u8g2.setDrawColor(1);
  }
  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

void drawRulesList(){
  u8g2.clearBuffer();
u8g2.setFont(u8g2_font_6x12_t_cyrillic);
  int n = (int)rules[selCh].size();
  for(int i=0;i<n;i++){
    auto &r = rules[selCh][i];
    char b[40];
    snprintf(b,sizeof(b),"%d:%s %s A:%u", i+1, actName(r.type), condName(r.kind), r.aUs);
    int y=12+i*9;
    if(i==ruleIdx){ u8g2.drawBox(0,y-7,128,9); u8g2.setDrawColor(0); }
    u8g2.drawUTF8(2,y,b);
    if(i==ruleIdx) u8g2.setDrawColor(1);
  }
  int y=12+n*9;
  if(ruleIdx==n){ u8g2.drawBox(0,y-7,128,9); u8g2.setDrawColor(0); }
  u8g2.drawUTF8(2,y,"+Добавить");
  if(ruleIdx==n) u8g2.setDrawColor(1);
  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

// Р’ EDIT РїРѕРєР°Р·С‹РІР°РµРј PWM РІС‹Р±СЂР°РЅРЅРѕРіРѕ РІС…РѕРґРЅРѕРіРѕ РєР°РЅР°Р»Р° (selCh) РІ РїСЂР°РІРѕРј РІРµСЂС…РЅРµРј СѓРіР»Сѓ
void drawEdit(){
  if(!edit) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_t_cyrillic);

  // PWM in corner
 /* char p[12];
  uint16_t us = pwmWidth[selCh];
  if(us>=900 && us<=2100) snprintf(p,sizeof(p),"РљР°РЅР°Р»%d:%u", selCh+1, us);
  else snprintf(p,sizeof(p),"РљР°РЅР°Р»%d:---", selCh+1);
  int tw = u8g2.getStrWidth(p);
  u8g2.drawUTF8(60-tw, 12, p);
*/

uint16_t us = pwmWidth[selCh];

char p[20];

if(us>=900 && us<=2100)
{
    snprintf(p,sizeof(p),"Ch%d: %uus", selCh+1, us);
}
else
{
    snprintf(p,sizeof(p),"Ch%d: ---", selCh+1);
}

u8g2.drawUTF8(2, 8, p);

// ---- signed value ----
if(us>=900 && us<=2100)
{
    float vs = pwm_to_signed(us);
    char s[16];
    snprintf(s,sizeof(s),"%.2f", vs);
    u8g2.drawUTF8(90, 8, s);

    // ---- draw bar ----
    int barW = 100;
    int barX = 14;
    int barY = 14;
    int barH = 6;

    u8g2.drawFrame(barX, barY, barW, barH);

    float v01 = pwm_to_01(us);
    int fill = (int)(v01 * (barW-2));
    u8g2.drawBox(barX+1, barY+1, fill, barH-2);
}
  if(edit->targetIndex <= 15 && us>=900 && us<=2100){
    float mappedOutUs = map_range_clamped((float)us, (float)edit->aUs, (float)edit->bUs, (float)edit->outAUs, (float)edit->outBUs);
    char m[20];
    snprintf(m,sizeof(m),"P:%4u", (unsigned)lroundf(mappedOutUs));
    u8g2.drawUTF8(88, 20, m);
  } else if(edit->targetIndex >= 16 && edit->targetIndex <= 19){
    char m[20];
    snprintf(m,sizeof(m),"B:%d..%d", edit->btsMinPct, edit->btsMaxPct);
    u8g2.drawUTF8(74, 20, m);
  }

  const int totalFields = 10;
  const int visibleFields = 4;
  int startField = clampi(fieldIdx - 1, 0, totalFields - visibleFields);
  char line[40];

  for(int row=0; row<visibleFields; row++){
    int f = startField + row;
    int y = 29 + row * 9;
    switch(f){
      case 0: snprintf(line,sizeof(line),"Тип: %s", actName(edit->type)); break;
      case 1: snprintf(line,sizeof(line),"Режим: %s", modeName(edit->mode)); break;
      case 2: snprintf(line,sizeof(line),"Условие: %s", condName(edit->kind)); break;
      case 3: snprintf(line,sizeof(line),"A: %u", edit->aUs); break;
      case 4: snprintf(line,sizeof(line),"B: %u", edit->bUs); break;
      case 5: snprintf(line,sizeof(line),"OutA: %u", edit->outAUs); break;
      case 6: snprintf(line,sizeof(line),"OutB: %u", edit->outBUs); break;
      case 7: snprintf(line,sizeof(line),"BTSmin: %d", edit->btsMinPct); break;
      case 8: snprintf(line,sizeof(line),"BTSmax: %d", edit->btsMaxPct); break;
      case 9: snprintf(line,sizeof(line),"Цель: %u", edit->targetIndex); break;
      default: line[0]=0; break;
    }
    if(fieldIdx==f) u8g2.drawBox(0,y-7,128,9);
    u8g2.setDrawColor(fieldIdx==f?0:1);
    u8g2.drawUTF8(2,y,line);
    u8g2.setDrawColor(1);
  }

  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

// ================== UI TASK ==================
static bool backDebounced(){
  static uint32_t t=0;
  if(halReadBackBtn()){
    uint32_t now=millis();
    if(now-t>200){ t=now; return true; }
  }
  return false;
}

void uiTask(void*){
  Serial.println("[UI] uiTask started");

  for(;;){
int d = halReadNavStep();  // в†ђ 1 С€Р°Рі = 1 С‰РµР»С‡РѕРє
  // рџ”Ќ DEBUG: СЃРјРѕС‚СЂРёРј, РµСЃС‚СЊ Р»Рё С€Р°РіРё
    if(d != 0){
      Serial.printf("ENC STEP = %d\n", d);
    }
 
bool click = encBtnClick();
bool back  = backDebounced();
if(click) Serial.println("CLICK");
if(back)  Serial.println("BACK");

    if(menu==MAIN){
      if(d>0) mainIdx++; else if(d<0) mainIdx--;
      mainIdx=clampi(mainIdx,0,3);
      if(click){
        if(mainIdx==0) menu=STATUSS;
        else if(mainIdx==1) menu=CHSEL;
        else if(mainIdx==2){ saveRules(); menu=SAVED; }
        else if(mainIdx==3){ loadRules(); menu=SAVED; }
      }
    } else if(menu==STATUSS){
      if(click || back) menu=MAIN;
    } else if(menu==CHSEL){
      if(d>0) chIdx++; else if(d<0) chIdx--;
      chIdx=clampi(chIdx,0,7);
      if(click){
        selCh=chIdx; ruleIdx=0;
        menu=RULES_LIST;
      }
      if(back) menu=MAIN;
    } else if(menu==RULES_LIST){
      int n=(int)rules[selCh].size();
      int maxIdx=n; // +Add
      if(d>0) ruleIdx++; else if(d<0) ruleIdx--;
      ruleIdx=clampi(ruleIdx,0,maxIdx);
      if(click){
        if(ruleIdx==n){
          rules[selCh].push_back(Rule{});
          edit=&rules[selCh].back();
          fieldIdx=0;
          menu=EDIT_RULE;
        } else {
          edit=&rules[selCh][ruleIdx];
          fieldIdx=0;
          menu=EDIT_RULE;
        }
      }
      if(back) menu=CHSEL;
    } else if(menu==EDIT_RULE){
      if(edit && d!=0){
        int step = (d>0)?10:-10;
        switch(fieldIdx){
          case 0: edit->type = (ActType)((((int)edit->type)+(d>0?1:2))%3); break;
          case 1: edit->mode = (Mode)((((int)edit->mode)+(d>0?1:3))%4); break;
          case 2: edit->kind = (Rule::CondKind)((((int)edit->kind)+(d>0?1:3))%4); break;
          case 3: edit->aUs  = clampi((int)edit->aUs + step, 900, 2100); break;
          case 4: edit->bUs  = clampi((int)edit->bUs + step, 900, 2500); break;
          case 5: edit->outAUs = clampi((int)edit->outAUs + step, 0, 4095); break;
          case 6: edit->outBUs = clampi((int)edit->outBUs + step, 0, 4095); break;
          case 7: edit->btsMinPct = clampi((int)edit->btsMinPct + (d>0?5:-5), -100, 100); break;
          case 8: edit->btsMaxPct = clampi((int)edit->btsMaxPct + (d>0?5:-5), -100, 100); break;
          case 9: edit->targetIndex = clampi((int)edit->targetIndex + (d>0?1:-1), 0, 26); break;
        }
      }
      if(click){
        fieldIdx++; if(fieldIdx>9) fieldIdx=0;
      }
      if(back) menu=RULES_LIST;
    } else if(menu==SAVED){
      if(click || back) menu=MAIN;
    }

    // render
    switch(menu){
      case MAIN: drawMain(); break;
      case STATUSS: drawStatus(); break;
      case CHSEL: drawCh(); break;
      case RULES_LIST: drawRulesList(); break;
      case EDIT_RULE: drawEdit(); break;
      case SAVED:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_t_cyrillic);
        u8g2.drawUTF8(30, 30, "Сохранено!");
        halI2CLock();
        u8g2.sendBuffer();
        halI2CUnlock();
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================== SETUP / LOOP ==================
void setup(){
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== PUM UI + PWM + RULE + OUTPUT (HAL) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.enableUTF8Print();
  halInit();      // MCP + PCA + BTS init
  initPWM();      // RMT inputs

  prefs.begin("cfg", false);
  loadRules();

  // demo rule if empty
  if(rules[0].empty()){
    Rule r;
    r.kind = Rule::ANY;
    r.mode = ENGINE;
    r.targetIndex = 0; // PCA0
    rules[0].push_back(r);
  }

  // tasks
  xTaskCreatePinnedToCore(pwmTask,   "PWM",  4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(logicTask, "LOGIC", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(outputTask,"OUT",  4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(debugTask, "DBG",  4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(uiTask,    "UI",   4096, nullptr, 1, nullptr, 1);

  webUiBegin(
    "ESP32-RC-CTRL",
    "12345678",
    buildStatusJson,
    buildRulesJson,
    applyRulesJson,
    saveRules,
    loadRules
  );

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);
  u8g2.drawUTF8(10, 30, "Готово!");
  halI2CLock();
  u8g2.sendBuffer();
  halI2CUnlock();
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}


