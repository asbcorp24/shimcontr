// hal.cpp
#include "hal.h"
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ================= MCP23017 =================
static Adafruit_MCP23X17 mcp;

// A port (0..7)
static const uint8_t MCP_RELAY[4] = {4, 3, 2, 1};  // A4..A1

// B port (8..15)
static const uint8_t MCP_REV_ON[3]  = {8+2, 8+4, 8+6}; // B2, B4, B6
static const uint8_t MCP_REV_POL[3] = {8+3, 8+5, 8+7}; // B3, B5, B7

// ================= PCA9685 =================
static Adafruit_PWMServoDriver pca(0x40);
static SemaphoreHandle_t gI2cMutex = nullptr;

// ================= BTS7960 =================
static const uint8_t BTS_RPWM[4] = {25, 27, 12, 32};
static const uint8_t BTS_LPWM[4] = {26, 14, 13, 33};

 

 
void halI2CLock()
{
    if (gI2cMutex) xSemaphoreTakeRecursive(gI2cMutex, portMAX_DELAY);
}

void halI2CUnlock()
{
    if (gI2cMutex) xSemaphoreGiveRecursive(gI2cMutex);
}

// ================= BUTTONS A/B =================
int halReadNavStep()
{
    static bool lastA = true;
    static bool lastB = true;
    static uint32_t lastMs = 0;

    halI2CLock();
    uint16_t gpio = mcp.readGPIOAB();
    halI2CUnlock();

    bool a = gpio & (1 << 8); // ENC A
    bool b = gpio & (1 << 9); // ENC B

    uint32_t now = millis();
    if (now - lastMs < 120) {   // антидребезг
        lastA = a;
        lastB = b;
        return 0;
    }

    int step = 0;

    // A = NEXT
    if (lastA && !a) {
        step = +1;
        lastMs = now;
    }

    // B = PREV
    if (lastB && !b) {
        step = -1;
        lastMs = now;
    }

    lastA = a;
    lastB = b;

    return step;
}

// =================================================
void halInit()
{
    Wire.begin();
    gI2cMutex = xSemaphoreCreateRecursiveMutex();

    halI2CLock();
    if (!mcp.begin_I2C()) {
        Serial.println("[HAL] MCP23017 init FAILED");
    } else {
        Serial.println("[HAL] MCP23017 OK");
    }

    // ---------- OUTPUTS ----------
    for (uint8_t i = 0; i < 4; i++) {
        mcp.pinMode(MCP_RELAY[i], OUTPUT);
        mcp.digitalWrite(MCP_RELAY[i], HIGH);
    }
    for (uint8_t i = 0; i < 3; i++) {
        mcp.pinMode(MCP_REV_ON[i], OUTPUT);
        mcp.pinMode(MCP_REV_POL[i], OUTPUT);
        mcp.digitalWrite(MCP_REV_ON[i], HIGH);
        mcp.digitalWrite(MCP_REV_POL[i], HIGH);
    }

    // ---------- INPUTS ----------
    mcp.pinMode(8, INPUT_PULLUP); // ENC A
    mcp.pinMode(9, INPUT_PULLUP); // ENC B
    mcp.pinMode(7, INPUT_PULLUP); // ENC BTN
    mcp.pinMode(5, INPUT_PULLUP); // BACK BTN

    // ---------- PCA9685 ----------
    pca.begin();
    pca.setPWMFreq(1000);
    halI2CUnlock();

    // ---------- BTS PWM ----------
    for (int i = 0; i < 4; i++) {
        int chR = i * 2;
        int chL = i * 2 + 1;
        ledcSetup(chR, 20000, 10);
        ledcSetup(chL, 20000, 10);
        ledcAttachPin(BTS_RPWM[i], chR);
        ledcAttachPin(BTS_LPWM[i], chL);
        ledcWrite(chR, 0);
        ledcWrite(chL, 0);
    }

    Serial.println("[HAL] Init complete");
}

// ================= RELAYS =================
void setRelay(uint8_t idx, bool on)
{
    if (idx >= 4) return;
    halI2CLock();
    mcp.digitalWrite(MCP_RELAY[idx], on ? HIGH : LOW);
    halI2CUnlock();
}

// ================= POLARITY RELAY =================
void setPolarityRelay(uint8_t idx, bool on, bool polarity)
{
    if (idx >= 3) return;
    halI2CLock();
    mcp.digitalWrite(MCP_REV_ON[idx], on ? HIGH : LOW);
    mcp.digitalWrite(MCP_REV_POL[idx], polarity ? HIGH : LOW);
    halI2CUnlock();
}

// ================= ENGINE PWM =================
void setEnginePwm(uint8_t idx, float duty)
{
    if (idx >= 16) return;
    duty = constrain(duty, 0.0f, 1.0f);
    uint16_t pwm12 = (uint16_t)(duty * 4095);
    halI2CLock();
    pca.setPWM(idx, 0, pwm12);
    halI2CUnlock();
}

// ================= BTS =================
void setBts(uint8_t idx, float value)
{
    if (idx >= 4) return;

    value = constrain(value, -1.0f, 1.0f);
    uint32_t duty = (uint32_t)(fabs(value) * 1023);

    int chR = idx * 2;
    int chL = idx * 2 + 1;

    if (value > 0.01f) {
        ledcWrite(chR, duty);
        ledcWrite(chL, 0);
    } else if (value < -0.01f) {
        ledcWrite(chR, 0);
        ledcWrite(chL, duty);
    } else {
        ledcWrite(chR, 0);
        ledcWrite(chL, 0);
    }
}

// ================= INPUTS =================
bool halReadEncA()    { halI2CLock(); bool v = (mcp.readGPIOAB() & (1 << 8)); halI2CUnlock(); return v; }
bool halReadEncB()    { halI2CLock(); bool v = (mcp.readGPIOAB() & (1 << 9)); halI2CUnlock(); return v; }
bool halReadEncBtn()  {
    static bool last = true;
    static uint32_t lastMs = 0;

    halI2CLock();
    uint16_t gpio = mcp.readGPIOAB();
    halI2CUnlock();
    bool cur = gpio & (1 << 7); // HIGH = отпущена

    uint32_t now = millis();
    if (now - lastMs < 150) {
        last = cur;
        return false;
    }

    bool clicked = (last == true && cur == false);
    if (clicked) lastMs = now;

    last = cur;
    return clicked;
}

bool halReadBackBtn() { halI2CLock(); bool v = (mcp.readGPIOAB() & (1 << 5)) == 0; halI2CUnlock(); return v; }
