#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <AiEsp32RotaryEncoder.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <vector>
#include "driver/rmt.h"

// ======================================================================
//                     ТИПЫ, СТРУКТУРЫ, ПРОТОТИПЫ
// ======================================================================

enum ActType { BTS, PCA_OUT, DAC_OUT, REVERSER };
enum Mode    { RELAY, POLARITY_RELAY, ENGINE, BTS_Mode };

struct Rule {
    enum CondKind { ANY, BELOW, ABOVE, BETWEEN } kind = ANY;
    uint16_t aUs = 1500, bUs = 2000;
    ActType type = PCA_OUT;
    uint8_t targetIndex = 0;
    Mode mode = RELAY;
};

// UI состояние
enum MenuState { MAIN, STATUSS, CHSEL, RULES, EDIT_RULE, SAVED };
MenuState menu = MAIN;

// Индексы по меню (РАЗДЕЛЁННЫЕ)
int mainIdx  = 0;   // для главного меню
int chIdx    = 0;   // выбор канала
int ruleIdx  = 0;   // выбор правила в списке
int fieldIdx = 0;   // выбор поля в редакторе

// Глобальные флаги редактора
Rule *edit = nullptr;

// Прототипы
void saveRules();
void loadRules();
void updatePWM();
void drawRules();
void drawStatus();
void drawMain();
void drawCh();
void drawEdit();
void uiTask(void *p);
void applyTypeDefaults(ActType t, Rule *r);
void IRAM_ATTR readEncoderISR();
bool isBackClicked();

// ======================================================================
//                               ПИНЫ
// ======================================================================

#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 33
#define ENC_B 32
#define ENC_BTN 15
#define BACK_BUTTON_PIN 4
#define ROTARY_VCC_PIN -1

#define PWM_INPUT_PIN1 19
#define PWM_INPUT_PIN2 18
#define PWM_INPUT_PIN3 5
#define PWM_INPUT_PIN4 17
#define PWM_INPUT_PIN5 16
#define PWM_INPUT_PIN6 2
#define PWM_INPUT_PIN7 23
#define PWM_INPUT_PIN8 34

// ======================================================================
//                         ГЛОБАЛЬНЫЕ PWM ПЕРЕМЕННЫЕ
// ======================================================================

uint16_t pwmWidth[8] = {0};
RingbufHandle_t rmt_rb[8];

rmt_channel_t rmtChannels[8] = {
    RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3,
    RMT_CHANNEL_4, RMT_CHANNEL_5, RMT_CHANNEL_6, RMT_CHANNEL_7
};

int pwmPins[8] = {
    PWM_INPUT_PIN1, PWM_INPUT_PIN2, PWM_INPUT_PIN3, PWM_INPUT_PIN4,
    PWM_INPUT_PIN5, PWM_INPUT_PIN6, PWM_INPUT_PIN7, PWM_INPUT_PIN8
};

const int PWM_COUNT = 8;

// ======================================================================
//                            UI / STORAGE
// ======================================================================

AiEsp32RotaryEncoder enc(ENC_A, ENC_B, ENC_BTN, ROTARY_VCC_PIN);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);
Preferences prefs;

String TargetArray[] = {
    "DAC1","DAC2","DAC3","DAC4","DAC5",
    "DAC6","DAC7","DAC8","DAC9","DAC10",
    "DAC11","DAC12","DAC13","DAC14","DAC15",
    "DAC16","BTS1","BTS2","BTS3","BTS4",
    "RELE1","RELE2","RELE3","RELE4",
    "REVERSE1","REVERSE2","REVERSE3","REVERSE4"
};
int selCh = 0;   // <---- ЭТОГО НЕ ХВАТАЛО
std::vector<Rule> rules[8];

// ======================================================================
//                            RMT PWM INIT
// ======================================================================
 

void pwmTask(void *p)
{
    Serial.println("[PWM] pwmTask started");

    static uint32_t lastUpdate[8] = {0};   // время последнего принятого импульса

    for (;;)
    {
        for (int ch = 0; ch < 8; ch++)
        {
            size_t item_size = 0;
            rmt_item32_t *item =
                (rmt_item32_t *)xRingbufferReceive(
                    rmt_rb[ch],
                    &item_size,
                    0   // НЕ БЛОКИРУЕМ
                );

            if (!item)
                continue;

            if (item_size >= sizeof(rmt_item32_t))
            {
                uint16_t us = 0;

                // Ищем HIGH импульс
                if (item[0].level0 == 1)
                    us = item[0].duration0;
                else if (item[0].level1 == 1)
                    us = item[0].duration1;

                // RC фильтр
                if (us >= 900 && us <= 2100)
                {
                    uint32_t now = millis();

                    // 🔥 ГЛАВНОЕ: обновляем не чаще 1 раза в 15 мс
                    if (now - lastUpdate[ch] >= 15)
                    {
                        pwmWidth[ch] = us;
                        lastUpdate[ch] = now;
                    }
                }
            }

            // ❗ ВСЕГДА возвращаем item
            vRingbufferReturnItem(rmt_rb[ch], item);
        }

        // Минимальная пауза, чтобы не душить CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void initPWM()
{
    for (int i = 0; i < 8; i++)
    {
        gpio_set_direction((gpio_num_t)pwmPins[i], GPIO_MODE_INPUT);
gpio_pullup_en((gpio_num_t)pwmPins[i]);
gpio_pulldown_dis((gpio_num_t)pwmPins[i]);

        rmt_config_t config = {};
        config.rmt_mode = RMT_MODE_RX;
        config.channel = rmtChannels[i];
        config.gpio_num = (gpio_num_t)pwmPins[i];
        config.mem_block_num = 1;
        config.clk_div = 80; // 1us tick при 80 MHz
        config.rx_config.filter_en = true;
        config.rx_config.filter_ticks_thresh = 10;
        config.rx_config.idle_threshold = 2500; // 30 ms

        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 8192, 0));
        ESP_ERROR_CHECK(rmt_get_ringbuf_handle(config.channel, &rmt_rb[i]));

        rmt_rx_start(config.channel, true);
    }

    Serial.println("[PWM] RMT PWM Ready");
}

void updatePWM()
{
    for (int ch = 0; ch < 8; ch++)
    {
        size_t item_size = 0;
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rmt_rb[ch], &item_size, 0);

        if (item && item_size >= sizeof(rmt_item32_t))
        {
            pwmWidth[ch] = item[0].duration0;
            vRingbufferReturnItem(rmt_rb[ch], item);
        }
    }
}

// ======================================================================
//                     UI + DRAW HELPERS
// ======================================================================

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

const char *actName(ActType t)
{
    switch (t)
    {
    case BTS:      return "BTS";
    case PCA_OUT:  return "PCA";
    case DAC_OUT:  return "DAC";
    case REVERSER: return "REV";
    }
    return "?";
}

const char *condName(Rule::CondKind c)
{
    switch (c)
    {
    case Rule::ANY:     return "ANY";
    case Rule::BELOW:   return "BELOW";
    case Rule::ABOVE:   return "ABOVE";
    case Rule::BETWEEN: return "BETWEEN";
    }
    return "?";
}

const char *ModeNames(Mode m)
{
    switch (m)
    {
    case RELAY:          return "RELAY";
    case POLARITY_RELAY: return "POLARITY";
    case ENGINE:         return "ENGINE";
    case BTS_Mode:       return "BTS";
    }
    return "?";
}

const char* menuName(MenuState m)
{
    switch (m)
    {
    case MAIN:      return "MAIN";
    case STATUSS:   return "STATUSS";
    case CHSEL:     return "CHSEL";
    case RULES:     return "RULES";
    case EDIT_RULE: return "EDIT_RULE";
    case SAVED:     return "SAVED";
    }
    return "?";
}

// ======================================================================
//                          SAVE / LOAD
// ======================================================================

void saveRules()
{
    Serial.println("[NVS] saveRules() called");
    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.createNestedArray("rules");

    for (int i = 0; i < 8; i++)
    {
        JsonArray chA = arr.createNestedArray();
        for (auto &r : rules[i])
        {
            JsonObject o = chA.createNestedObject();
            o["type"] = (int)r.type;
            o["cond"] = (int)r.kind;
            o["a"]    = r.aUs;
            o["b"]    = r.bUs;
            o["tgt"]  = r.targetIndex;
            o["mode"] = (int)r.mode;
        }
    }

    String s;
    serializeJson(doc, s);
    prefs.putString("cfg", s);

    Serial.print("[NVS] Saved, length = ");
    Serial.println(s.length());
}

void loadRules()
{
    Serial.println("[NVS] loadRules() called");

    if (!prefs.isKey("cfg")) {
        Serial.println("[NVS] Key cfg not found, using defaults");
        return;
    }

    String s = prefs.getString("cfg");
    DynamicJsonDocument doc(4096);

    auto err = deserializeJson(doc, s);
    if (err)
    {
        Serial.print("[NVS] JSON error: ");
        Serial.println(err.c_str());
        return;
    }

    JsonArray arr = doc["rules"];

    for (int i = 0; i < 8 && i < (int)arr.size(); i++)
    {
        rules[i].clear();
        for (JsonObject o : arr[i].as<JsonArray>())
        {
            Rule r;
            r.type        = (ActType)(int)o["type"];
            r.kind        = (Rule::CondKind)(int)o["cond"];
            r.aUs         = o["a"];
            r.bUs         = o["b"];
            r.targetIndex = o["tgt"];
            r.mode        = (Mode)(int)o["mode"];
            rules[i].push_back(r);
        }
        Serial.printf("[NVS] Channel %d: loaded %d rules\n", i, (int)rules[i].size());
    }

    Serial.println("[NVS] Loaded");
}

// ======================================================================
//                        DRAW STATUS
// ======================================================================

void drawStatus()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);

    const int barWidth   = 10;
    const int barSpacing = 6;
    const int baseY      = 52;   // линия "нуля"
    const int maxHeight  = 40;   // высота столбца

    for (int i = 0; i < 8; i++)
    {
        int x = 4 + i * (barWidth + barSpacing);

        int h = 0;
        if (pwmWidth[i] >= 900 && pwmWidth[i] <= 2100)
        {
            h = map(pwmWidth[i], 1000, 2000, 2, maxHeight);
            h = clampi(h, 2, maxHeight);
        }

        // Столбец
        if (h > 0)
        {
            u8g2.drawBox(x, baseY - h, barWidth, h);
        }

        // Число сверху
        char label[8];
        if (pwmWidth[i] > 0)
            sprintf(label, "%d", pwmWidth[i]);
        else
            strcpy(label, "--");

        u8g2.drawStr(x - 2, 63, label);
    }

    // линия базы
    u8g2.drawHLine(0, baseY, 128);

    u8g2.sendBuffer();
}


// ======================================================================
//                          MAIN MENU
// ======================================================================

void drawMain()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    const char *m[] = {"Status", "Edit rules", "Save", "Load"};

    for (int i = 0; i < 4; i++)
    {
        int y = 20 + i * 12;

        if (i == mainIdx)
        {
            u8g2.drawBox(0, y - 10, 128, 12);
            u8g2.setDrawColor(0);
        }

        u8g2.drawStr(4, y, m[i]);

        if (i == mainIdx)
            u8g2.setDrawColor(1);
    }

    u8g2.sendBuffer();
}

// ======================================================================
//                        CHANNEL SELECT
// ======================================================================

void drawCh()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    for (int i = 0; i < 8; i++)
    {
        char b[30];
        sprintf(b, "CH%d (%d)", i + 1, (int)rules[i].size());

        int y = 20 + i * 12;

        if (i == chIdx)
        {
            u8g2.drawBox(0, y - 10, 128, 12);
            u8g2.setDrawColor(0);
        }

        u8g2.drawStr(4, y, b);

        if (i == chIdx)
            u8g2.setDrawColor(1);
    }

    u8g2.sendBuffer();
}

// ======================================================================
//                           RULE LIST
// ======================================================================

void drawRules()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);

    int n = rules[selCh].size();   // selCh установлен в CHSEL

    for (int i = 0; i < n; i++)
    {
        auto &r = rules[selCh][i];

        char b[40];
        sprintf(b, "%d: %s %s A:%d", i + 1, actName(r.type), condName(r.kind), r.aUs);

        int y = 12 + i * 9;

        if (i == ruleIdx)
        {
            u8g2.drawBox(0, y - 7, 128, 9);
            u8g2.setDrawColor(0);
        }

        u8g2.drawStr(2, y, b);

        if (i == ruleIdx)
            u8g2.setDrawColor(1);
    }

    int y = 12 + n * 9;

    if (ruleIdx == n)
    {
        u8g2.drawBox(0, y - 7, 128, 9);
        u8g2.setDrawColor(0);
    }

    u8g2.drawStr(2, y, "+Add");

    if (ruleIdx == n)
        u8g2.setDrawColor(1);

    u8g2.sendBuffer();
}

// ======================================================================
//                       RULE EDITOR (вариант 2)
// ======================================================================

void applyTypeDefaults(ActType t, Rule *r)
{
    switch (t)
    {
    case BTS:
        r->mode        = BTS_Mode;
        r->kind        = Rule::ABOVE;
        r->aUs         = 1500;
        r->bUs         = 2000;
        r->targetIndex = 0;
        break;

    case PCA_OUT:
        r->mode        = POLARITY_RELAY;
        r->kind        = Rule::BELOW;
        r->aUs         = 1200;
        r->bUs         = 2200;
        r->targetIndex = 3;
        break;

    case DAC_OUT:
        r->mode        = ENGINE;
        r->kind        = Rule::BETWEEN;
        r->aUs         = 1300;
        r->bUs         = 2400;
        r->targetIndex = 5;
        break;

    case REVERSER:
        r->mode        = ENGINE;
        r->kind        = Rule::BETWEEN;
        r->aUs         = 1350;
        r->bUs         = 2300;
        r->targetIndex = 1;
        break;
    }
}

void drawEdit()
{
    if (!edit)
        return;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);

    char line[40];

    // 0: Type
    sprintf(line, "Type: %s", actName(edit->type));
    if (fieldIdx == 0) u8g2.drawBox(0, 3, 128, 10);
    u8g2.setDrawColor(fieldIdx == 0 ? 0 : 1);
    u8g2.drawStr(2, 10, line);
    u8g2.setDrawColor(1);

    // 1: Mode
    sprintf(line, "Mode: %s", ModeNames(edit->mode));
    if (fieldIdx == 1) u8g2.drawBox(0, 13, 128, 10);
    u8g2.setDrawColor(fieldIdx == 1 ? 0 : 1);
    u8g2.drawStr(2, 20, line);
    u8g2.setDrawColor(1);

    // 2: Cond
    sprintf(line, "Cond: %s", condName(edit->kind));
    if (fieldIdx == 2) u8g2.drawBox(0, 23, 128, 10);
    u8g2.setDrawColor(fieldIdx == 2 ? 0 : 1);
    u8g2.drawStr(2, 30, line);
    u8g2.setDrawColor(1);

    // 3: A
    sprintf(line, "A: %d", edit->aUs);
    if (fieldIdx == 3) u8g2.drawBox(0, 33, 128, 10);
    u8g2.setDrawColor(fieldIdx == 3 ? 0 : 1);
    u8g2.drawStr(2, 40, line);
    u8g2.setDrawColor(1);

    // 4: B
    sprintf(line, "B: %d", edit->bUs);
    if (fieldIdx == 4) u8g2.drawBox(0, 43, 128, 10);
    u8g2.setDrawColor(fieldIdx == 4 ? 0 : 1);
    u8g2.drawStr(2, 50, line);
    u8g2.setDrawColor(1);

    // 5: Target
    sprintf(line, "Target: %s", TargetArray[edit->targetIndex]);
    if (fieldIdx == 5) u8g2.drawBox(0, 53, 128, 10);
    u8g2.setDrawColor(fieldIdx == 5 ? 0 : 1);
    u8g2.drawStr(2, 60, line);
    u8g2.setDrawColor(1);

    u8g2.sendBuffer();
}

// ======================================================================
//                      BACK BUTTON
// ======================================================================

bool isBackClicked()
{
    static uint32_t lastPress = 0;

    if (!digitalRead(BACK_BUTTON_PIN))
    {
        if (millis() - lastPress > 200)
        {
            lastPress = millis();
            Serial.println("[BTN] BACK pressed");
            return true;
        }
    }
    return false;
}

// ======================================================================
//                           ENCODER ISR
// ======================================================================

void IRAM_ATTR readEncoderISR()
{
    enc.readEncoder_ISR();
}




void printPwmDebug()
{
    Serial.print(" | PWM: ");
    for (int i = 0; i < 8; i++)
    {
        if (pwmWidth[i] < 800 || pwmWidth[i] > 2500)
            Serial.print("0* ");
        else
            Serial.printf("%4d ", pwmWidth[i]);
    }
}


// ======================================================================
//                                UI TASK
// ======================================================================

void uiTask(void *p)
{
    Serial.println("[UI] Task started");
    for (;;)
    {
        static int lastValue = 0;
        int value = enc.readEncoder();
        int delta = value - lastValue;
        lastValue = value;

        bool btnClicked = enc.isEncoderButtonClicked();
        if (delta != 0 || btnClicked)
        {
         Serial.printf(
    "[UI] Menu=%s d=%+d btn=%d main=%d ch=%d rule=%d field=%d selCh=%d",
    menuName(menu),
    delta,
    btnClicked,
    mainIdx,
    chIdx,
    ruleIdx,
    fieldIdx,
    selCh
);

printPwmDebug();
Serial.println();
        }

        switch (menu)
        {
        // ---------------------------------------------------------
        // MAIN MENU
        // ---------------------------------------------------------
        case MAIN:
            if (delta > 0)      mainIdx++;
            else if (delta < 0) mainIdx--;

            mainIdx = clampi(mainIdx, 0, 3);

            if (btnClicked)
            {
                Serial.printf("[UI] MAIN select index %d\n", mainIdx);
                if (mainIdx == 0) {
                    menu = STATUSS;
                }
                else if (mainIdx == 1) {
                    menu = CHSEL;
                }
                else if (mainIdx == 2) {
                    saveRules();
                    menu = SAVED;
                }
                else if (mainIdx == 3) {
                    loadRules();
                    menu = SAVED;
                }
            }
            break;

        // ---------------------------------------------------------
        // PWM STATUS
        // ---------------------------------------------------------
        case STATUSS:
            if (delta != 0)
            {
                // Просто игнорируем прокрутку здесь
            }
            if (btnClicked)
            {
                // Нажатие в STATUSS — возврат в MAIN
                Serial.println("[UI] STATUSS click -> MAIN");
                menu = MAIN;
            }
        
            break;

        // ---------------------------------------------------------
        // SELECT CHANNEL
        // ---------------------------------------------------------
        case CHSEL:
            if (delta > 0)      chIdx++;
            else if (delta < 0) chIdx--;

            chIdx = clampi(chIdx, 0, 7);

            if (btnClicked)
            {
                selCh = chIdx;
                ruleIdx = 0;
                Serial.printf("[UI] CHSEL: selCh=%d -> RULES\n", selCh);
                menu = RULES;
            }
            break;

        // ---------------------------------------------------------
        // RULE LIST
        // ---------------------------------------------------------
        case RULES:
        {
            int n = rules[selCh].size(); // правил
            int maxIdx = n;              // последний — "+Add"

            if (delta > 0)      ruleIdx++;
            else if (delta < 0) ruleIdx--;

            ruleIdx = clampi(ruleIdx, 0, maxIdx);

            if (btnClicked)
            {
                if (ruleIdx == n)  // "+Add"
                {
                    Rule r;
                    rules[selCh].push_back(r);
                    Serial.printf("[UI] RULES: add rule, now count=%d\n", (int)rules[selCh].size());
                }
                else
                {
                    edit = &rules[selCh][ruleIdx];
                    fieldIdx = 0;
                    Serial.printf("[UI] RULES: edit rule %d -> EDIT_RULE\n", ruleIdx);
                    menu = EDIT_RULE;
                }
            }
        }
            break;

        // ---------------------------------------------------------
        // EDIT RULE (Вариант 2)
        // ---------------------------------------------------------
        case EDIT_RULE:
            if (edit)
            {
                // Вариант 2:
                // - прокрутка меняет значение текущего поля
                // - кнопка переходит к следующему полю
                if (delta != 0)
                {
                    Serial.printf("[UI] EDIT_RULE: delta=%d on fieldIdx=%d\n", delta, fieldIdx);
                    int step = (delta > 0) ? 10 : -10;

                    switch (fieldIdx)
                    {
                    case 0: // Type
                        if (delta > 0)
                            edit->type = (ActType)(((int)edit->type + 1) % 4);
                        else
                            edit->type = (ActType)(((int)edit->type + 3) % 4);
                        applyTypeDefaults(edit->type, edit);
                        Serial.printf("[UI] EDIT_RULE: type=%d\n", (int)edit->type);
                        break;

                    case 1: // Mode
                        if (delta > 0)
                            edit->mode = (Mode)(((int)edit->mode + 1) % 4);
                        else
                            edit->mode = (Mode)(((int)edit->mode + 3) % 4);
                        Serial.printf("[UI] EDIT_RULE: mode=%d\n", (int)edit->mode);
                        break;

                    case 2: // Cond
                        if (delta > 0)
                            edit->kind = (Rule::CondKind)(((int)edit->kind + 1) % 4);
                        else
                            edit->kind = (Rule::CondKind)(((int)edit->kind + 3) % 4);
                        Serial.printf("[UI] EDIT_RULE: cond=%d\n", (int)edit->kind);
                        break;

                    case 3: // A
                        edit->aUs = clampi(edit->aUs + step, 1000, 2000);
                        Serial.printf("[UI] EDIT_RULE: aUs=%d\n", edit->aUs);
                        break;

                    case 4: // B
                        edit->bUs = clampi(edit->bUs + step, 1000, 2500);
                        Serial.printf("[UI] EDIT_RULE: bUs=%d\n", edit->bUs);
                        break;

                    case 5: // Target
                        if (delta > 0)
                            edit->targetIndex = clampi(edit->targetIndex + 1, 0, 27);
                        else
                            edit->targetIndex = clampi(edit->targetIndex - 1, 0, 27);
                        Serial.printf("[UI] EDIT_RULE: targetIndex=%d\n", edit->targetIndex);
                        break;
                    }
                }

                if (btnClicked)
                {
                    fieldIdx++;
                    if (fieldIdx > 5) fieldIdx = 0;
                    Serial.printf("[UI] EDIT_RULE: next fieldIdx=%d\n", fieldIdx);
                }
            }
            break;

        // ---------------------------------------------------------
        // SAVED SCREEN
        // ---------------------------------------------------------
        case SAVED:
            if (btnClicked)
            {
                Serial.println("[UI] SAVED -> MAIN");
                menu = MAIN;
            }
            break;
        }

        // ---------------------------------------
        // BACK BUTTON (общий)
        // ---------------------------------------
        if (isBackClicked())
        {
            Serial.printf("[UI] BACK from %s\n", menuName(menu));
            switch (menu)
            {
            case EDIT_RULE:
                menu = RULES;
                break;

            case RULES:
                menu = CHSEL;
                break;

            case CHSEL:
            case STATUSS:
            case SAVED:
                menu = MAIN;
                break;

            default:
                break;
            }
        }

        // ---------------------------------------
        // RENDER
        // ---------------------------------------
        switch (menu)
        {
        case MAIN:
            drawMain();
            break;

        case STATUSS:
            drawStatus();
            break;

        case CHSEL:
            drawCh();
            break;

        case RULES:
            drawRules();
            break;

        case EDIT_RULE:
            drawEdit();
            break;

        case SAVED:
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_6x12_tr);
            u8g2.drawStr(40, 30, "Saved!");
            u8g2.sendBuffer();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ======================================================================
//                            SETUP / LOOP
// ======================================================================

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== START SHIM WITH DEBUG ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    u8g2.begin();

    enc.begin();
    enc.setup(readEncoderISR);
    enc.setBoundaries(-10000, 10000, false);
    enc.setAcceleration(150);

    pinMode(ENC_BTN, INPUT_PULLUP);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);

    initPWM();
xTaskCreatePinnedToCore(
    pwmTask,
    "PWM",
    2048,
    nullptr,
    3,      // приоритет ВЫШЕ UI
    nullptr,
    0       // Core 0
);

    prefs.begin("cfg", false);
    loadRules();

    if (rules[0].empty())
    {
        Rule demo;
        rules[0].push_back(demo);
        Serial.println("[INIT] rules[0] was empty, added demo rule");
    }

    xTaskCreatePinnedToCore(uiTask, "UI", 4096, nullptr, 1, nullptr, 1);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(10, 30, "UI EDITOR READY");
    u8g2.sendBuffer();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));
}
