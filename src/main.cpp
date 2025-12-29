#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Инициализация MCP23017
  if (!mcp.begin_I2C()) {
    Serial.println("[ERROR] MCP23017 initialization failed.");
    while (1); // Ожидание, если инициализация не удалась
  }
  
  Serial.println("[INFO] MCP23017 initialized.");
  
  // Настройка всех пинов как входы
  for (uint8_t pin = 0; pin < 16; pin++) {
mcp.pinMode(pin, INPUT_PULLUP); // ✅ включает внутренний подтягивающий резистор
  }
  
  Serial.println("[INFO] All pins set to INPUT with pull-ups.");
}

void loop() {
  // Проверка всех пинов MCP23017
  for (uint8_t pin = 0; pin < 16; pin++) {
    bool pinState = mcp.digitalRead(pin);
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.println(pinState ? "HIGH" : "LOW");
  }
  
  delay(1000);  // Пауза 1 секунда между проверками
}
