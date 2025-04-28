#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "BluetoothSerial.h"

// Pines de motor (DRV8833)
const int motorA1 = 26;  // Motor izquierdo A IN1
const int motorA2 = 25;  // Motor izquierdo A IN2
const int motorB1 = 32;  // Motor derecho B IN3
const int motorB2 = 33;  // Motor derecho B IN4

// LED de estado (opcional): usa el LED integrado del ESP32
const int ledPin   = 2;

// Ajustes de PWM
const int pwmFreq = 5000;      // 5 kHz
// MCPWM unidades y timers
#define MCPWM_UNIT_A    MCPWM_UNIT_0
#define MCPWM_TIMER_A   MCPWM_TIMER_0
#define MCPWM_UNIT_B    MCPWM_UNIT_0
#define MCPWM_TIMER_B   MCPWM_TIMER_1

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_motores");  // Nombre Bluetooth

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Inicializar MCPWM para Motor A (izquierdo)
  mcpwm_gpio_init(MCPWM_UNIT_A, MCPWM0A, motorA1);
  mcpwm_gpio_init(MCPWM_UNIT_A, MCPWM0B, motorA2);
  mcpwm_config_t confA = {
    .frequency = pwmFreq,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .counter_mode = MCPWM_UP_COUNTER,
    .duty_mode = MCPWM_DUTY_MODE_0,
  };
  mcpwm_init(MCPWM_UNIT_A, MCPWM_TIMER_A, &confA);

  // Inicializar MCPWM para Motor B (derecho)
  mcpwm_gpio_init(MCPWM_UNIT_B, MCPWM1A, motorB1);
  mcpwm_gpio_init(MCPWM_UNIT_B, MCPWM1B, motorB2);
  mcpwm_init(MCPWM_UNIT_B, MCPWM_TIMER_B, &confA);
}

// Ajusta velocidad del Motor A (izquierdo): duty = -100…100 (%)
void setMotorA(int duty) {
  duty = constrain(duty, -100, 100);
  if (duty > 0) {
    mcpwm_set_duty(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_A, duty);
    mcpwm_set_duty(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_B, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_B, -duty);
  }
  mcpwm_set_duty_type(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_A, MCPWM_TIMER_A, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

// Ajusta velocidad del Motor B (derecho): duty = -100…100 (%)
void setMotorB(int duty) {
  duty = constrain(duty, -100, 100);
  if (duty > 0) {
    mcpwm_set_duty(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_A, duty);
    mcpwm_set_duty(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_B, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_B, -duty);
  }
  mcpwm_set_duty_type(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_B, MCPWM_TIMER_B, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void loop() {
  // Si llegan al menos 5 bytes por Bluetooth
  if (SerialBT.available() >= 5) {
    if (SerialBT.read() == 'H') {
      // Leer velocidad derecha (wr) y izquierda (wl)
      int16_t wr =  (int16_t)SerialBT.read() | ((int16_t)SerialBT.read() << 8);
      int16_t wl =  (int16_t)SerialBT.read() | ((int16_t)SerialBT.read() << 8);

      // Mostrar por puerto serie
      Serial.printf("wr = %d, wl = %d\n", wr, wl);

      // Señal de recepción
      digitalWrite(ledPin, HIGH);

      // Convertir a porcentaje [-100…100]
      bool wrNeg = (wr < 0);
      bool wlNeg = (wl < 0);
      uint16_t awr = abs(wr);
      uint16_t awl = abs(wl);
      // Mapear rango 0…255 a 0…100%
      int dutyWr = map(min(awr, (uint16_t)255), 0, 255, 0, 100);
      int dutyWl = map(min(awl, (uint16_t)255), 0, 255, 0, 100);
      if (wrNeg) dutyWr = -dutyWr;
      if (wlNeg) dutyWl = -dutyWl;

      // Ajustar motores
      setMotorB(dutyWr);
      setMotorA(dutyWl);

      // Apagar LED
      digitalWrite(ledPin, LOW);
    } else {
      // Si no es 'H', descartamos byte
      SerialBT.read();
    }
  }
}
