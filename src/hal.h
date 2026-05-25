#pragma once
#include <Arduino.h>

// ===================== I2C BUS LOCK =====================
void halI2CLock();
void halI2CUnlock();

// ===================== INIT =====================
void halInit();
int halReadNavStep();
// ===================== RELAY =====================
void setRelay(uint8_t ch, bool on);

// ===================== POLARITY RELAY =====================
void setPolarityRelay(uint8_t ch, bool on, bool polarity);

// ===================== ENGINE (PWM) =====================
void setEnginePwm(uint8_t ch, float value); // 0.0 ... 1.0

// ===================== BTS =====================
void setBts(uint8_t ch, float value); // -1.0 ... +1.0
// ===== inputs from MCP23017 =====
bool halReadEncA();      // encoder A
bool halReadEncB();      // encoder B
bool halReadEncBtn();    // encoder button
bool halReadBackBtn();   // back button
