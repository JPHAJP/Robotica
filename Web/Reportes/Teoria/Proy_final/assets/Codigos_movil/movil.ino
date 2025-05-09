#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "BluetoothSerial.h"

// Pines de motor (DRV8833)
const int motorA1 = 26;  // Motor izquierdo IN1
const int motorA2 = 25;  // Motor izquierdo IN2
const int motorB1 = 32;  // Motor derecho IN3
const int motorB2 = 33;  // Motor derecho IN4

// LED de estado (recibe paquete)
const int ledPin   = 2;

// Frecuencia de PWM
const int pwmFreq = 5000;      // 5 kHz

// MCPWM timers
#define MCPWM_TIMER_A   MCPWM_TIMER_0
#define MCPWM_TIMER_B   MCPWM_TIMER_1

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_motores");  // Nombre Bluetooth

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Inicializar MCPWM para Motor A (izquierdo)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motorA2);
// --- Configuración correcta de MCPWM ---
mcpwm_config_t confA = {
  .frequency    = pwmFreq,              // 1º campo
  .cmpr_a       = 0,                    // 2º campo
  .cmpr_b       = 0,                    // 3º campo
  .duty_mode    = MCPWM_DUTY_MODE_0,    // 4º campo (antes estaba 5º)
  .counter_mode = MCPWM_UP_COUNTER      // 5º campo (antes estaba 4º)
};

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_A, &confA);

  // Inicializar MCPWM para Motor B (derecho)
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, motorB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, motorB2);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_B, &confA);
}

// Ajusta velocidad del Motor A (izquierdo): duty = -100…100 (%)
void setMotorA(int duty) {
  duty = constrain(duty, -100, 100);
  if (duty > 0) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_A, duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_B, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_B, -duty);
  }
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_A, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

// Ajusta velocidad del Motor B (derecho): duty = -100…100 (%)
void setMotorB(int duty) {
  duty = constrain(duty, -100, 100);
  if (duty > 0) {
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_A, duty);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_B, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_B, -duty);
  }
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_B, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void loop() {
  // Esperamos paquete de 5 bytes: 'H' + wr(lo,hi) + wl(lo,hi)
  if (SerialBT.available() >= 5) {
    if (SerialBT.read() == 'H') {
      int16_t wr =  (int16_t)SerialBT.read() | ((int16_t)SerialBT.read() << 8);
      int16_t wl =  (int16_t)SerialBT.read() | ((int16_t)SerialBT.read() << 8);

      // Parpadeo indicador
      digitalWrite(ledPin, HIGH);

      // Convertir rangos 0…255 → 0…100%
      bool wrNeg = wr < 0;
      bool wlNeg = wl < 0;
      uint16_t awr = abs(wr), awl = abs(wl);
      int dutyWr = map(min(awr, (uint16_t)255), 0, 255, 0, 100);
      int dutyWl = map(min(awl, (uint16_t)255), 0, 255, 0, 100);
      if (wrNeg) dutyWr = -dutyWr;
      if (wlNeg) dutyWl = -dutyWl;

      // Aplicar a motores
      setMotorB(dutyWr);
      setMotorA(dutyWl);

      digitalWrite(ledPin, LOW);
    } else {
      // descartar si no es 'H'
      SerialBT.read();
    }
  }
}
