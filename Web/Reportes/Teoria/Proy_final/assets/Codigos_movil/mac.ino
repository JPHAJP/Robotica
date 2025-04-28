#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_system.h"

BluetoothSerial SerialBT;

void setup() {
  // Inicializamos Serial USB
  Serial.begin(115200);
  delay(100);

  // Iniciamos Bluetooth (nombre arbitrario)
  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Error al inicializar Bluetooth");
    while (1) { delay(1000); }
  }

  // Leemos la MAC de Bluetooth (ESP_MAC_BT)
  uint8_t macBt[6];
  esp_read_mac(macBt, ESP_MAC_BT);

  // Formateamos como XX:XX:XX:XX:XX:XX
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          macBt[0], macBt[1], macBt[2],
          macBt[3], macBt[4], macBt[5]);

  // Mostramos por USB
  Serial.print("MAC Bluetooth: ");
  Serial.println(macStr);
}

void loop() {
  // No hacemos nada más
}
