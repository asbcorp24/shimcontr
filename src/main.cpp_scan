#include <Wire.h>
#include <Arduino.h>
void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  
  // Инициализация шины I2C (SDA = 21, SCL = 22 для ESP32)
  Wire.begin(21, 22); 
  
  Serial.println("Перебор всех I2C устройств...");
}

void loop() {
  // Переменная для хранения найденных адресов
  String foundDevices = "";

  // Перебор всех возможных адресов I2C (от 1 до 127)
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      // Если устройство отвечает, добавляем его в список
      foundDevices += "Устройство найдено на адресе: 0x";
      foundDevices += String(address, HEX);
      foundDevices += "\n";
    } else if (error == 4) {
      // Если ошибка 4, это значит, что устройство не отвечает, но ошибка не критична
      foundDevices += "Ошибка на адресе: 0x";
      foundDevices += String(address, HEX);
      foundDevices += "\n";
    }
  }

  // Если найдено хотя бы одно устройство, выводим список
  if (foundDevices.length() > 0) {
    Serial.print(foundDevices); // Выводим найденные адреса в Serial Monitor
  } else {
    Serial.println("Устройства не найдены.");
  }

  // Задержка перед следующей проверкой
  delay(5000); // Ожидаем 5 секунд перед следующим циклом
}
