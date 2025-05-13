#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_mac.h"
#include "esp_system.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Error al inicializar Bluetooth");
    while (true) { delay(1000); }
  }

  uint8_t macBt[6];
  // ESP_MAC_BT está definido en esp_mac.h
  esp_read_mac(macBt, ESP_MAC_BT);

  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          macBt[0], macBt[1], macBt[2],
          macBt[3], macBt[4], macBt[5]);

  Serial.print("MAC Bluetooth: ");
  Serial.println(macStr);
}

void loop() {}



// ESP USB C
// MAC Bluetooth: 88:13:BF:70:40:72
// PWM mínimo con movimiento: 180 de 255