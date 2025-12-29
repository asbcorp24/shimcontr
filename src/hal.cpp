// hal.cpp
#include "hal.h"
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_PWMServoDriver.h>

// ================= MCP23017 =================
static Adafruit_MCP23X17 mcp;

// A port (0..7)
static const uint8_t MCP_RELAY[4] = {4, 3, 2, 1};  // A4..A1

// B port (8..15)
static const uint8_t MCP_REV_ON[3]  = {8+2, 8+4, 8+6}; // B2, B4, B6
static const uint8_t MCP_REV_POL[3] = {8+3, 8+5, 8+7}; // B3, B5, B7

// ================= PCA9685 =================
static Adafruit_PWMServoDriver pca(0x40);

// ================= BTS7960 =================
static const uint8_t BTS_RPWM[4] = {25, 27, 12, 32};
static const uint8_t BTS_LPWM[4] = {26, 14, 13, 33};

// =================================================

void halInit()
{
   
    Wire.begin();

    // MCP23017 Инициализация
    if (!mcp.begin_I2C()) {
        Serial.println("[HAL] MCP23017 begin_I2C FAILED");
    } else {
        Serial.println("[HAL] MCP23017 started");
    }

    // Настроим все пины расширителя как выходы (0..15)
    for (uint8_t pin = 0; pin < 16; pin++) {
        mcp.pinMode(pin, OUTPUT);
        mcp.digitalWrite(pin, HIGH);
    }

    // PCA9685
    pca.begin();
    pca.setPWMFreq(1000);

    // BTS PWM (LEDC)
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

    Serial.println("[HAL] Initialization complete");
}

// ================= RELAYS =================

void setRelay(uint8_t idx, bool on)
{
    if (idx >= 4) return;
    mcp.digitalWrite(MCP_RELAY[idx], on ? HIGH : LOW);
}

// ===== POLARITY RELAY =====
// polarity: true == forward, false == reverse
void setPolarityRelay(uint8_t idx, bool on, bool polarity)
{
    if (idx >= 3) return;

    // Включаем "ON" реле
    mcp.digitalWrite(MCP_REV_ON[idx], on ? HIGH : LOW);
    // Направление
    mcp.digitalWrite(MCP_REV_POL[idx], polarity ? HIGH : LOW);
}

// ===== ENGINE PWM =====
void setEnginePwm(uint8_t idx, float duty)
{
    if (idx >= 16) return;
    duty = constrain(duty, 0.0f, 1.0f);
    uint16_t pwm12 = (uint16_t)(duty * 4095);
    pca.setPWM(idx, 0, pwm12);
}

// ===== BTS7960 =====
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
// ========== INPUT FUNCTIONS (MCP23017 buttons & encoder) ==========

bool halReadEncA() {
    bool state = mcp.digitalRead(8 + 0) == HIGH;
    Serial.print("Enc A: ");
    Serial.println(state);
    return state;
}

bool halReadEncB() {
    bool state = mcp.digitalRead(8 + 1) == HIGH;
    Serial.print("Enc B: ");
    Serial.println(state);
    return state;
}

bool halReadEncBtn() {
    bool state = mcp.digitalRead(7) == LOW;
    Serial.print("Enc Btn: ");
    Serial.println(state);
    return state;
}

bool halReadBackBtn() {
    // MCP back button на A5 — pressed = LOW
    return mcp.digitalRead(5) == LOW;
}
