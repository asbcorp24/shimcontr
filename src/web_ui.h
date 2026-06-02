#pragma once

#include <Arduino.h>

typedef String (*WebUiGetJsonFn)();
typedef bool (*WebUiSetJsonFn)(const String& json);
typedef void (*WebUiActionFn)();

void webUiBegin(
  const char* apSsid,
  const char* apPass,
  bool enabled,
  WebUiGetJsonFn getStatusJson,
  WebUiGetJsonFn getRulesJson,
  WebUiSetJsonFn setRulesJson,
  WebUiActionFn saveRulesFn,
  WebUiActionFn loadRulesFn
);

void webUiSetEnabled(bool enabled);
