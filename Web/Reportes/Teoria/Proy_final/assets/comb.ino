#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "BluetoothSerial.h"
#include <AccelStepper.h>

// Comprobación de compatibilidad con Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no está habilitado! Por favor ejecuta `make menuconfig` para habilitarlo
#endif

BluetoothSerial SerialBT;

// ===== CONFIGURACIÓN PARA MOTORES DE MOVIMIENTO (MÓVIL) =====
// Pines de motor (DRV8833)
const int motorA1 = 26;  // Motor izquierdo IN1
const int motorA2 = 25;  // Motor izquierdo IN2
const int motorB1 = 32;  // Motor derecho IN3
const int motorB2 = 33;  // Motor derecho IN4

// LED de estado (recibe paquete)
const int ledPin = 2;

// Frecuencia de PWM
const int pwmFreq = 5000;  // 5 kHz

// MCPWM timers
#define MCPWM_TIMER_A   MCPWM_TIMER_0
#define MCPWM_TIMER_B   MCPWM_TIMER_1

// ===== CONFIGURACIÓN PARA MOTORES ARTICULADOS =====
#define motorInterfaceType 1

// Definición de estructura para los motores
struct StepperMotor {
  AccelStepper stepper;
  int stepPin;
  int dirPin;
  int sensorPin;
  int pasosPorVuelta;
  float velocidadNormal;
  float aceleracionNormal;
  int motorID;
};

// Parámetros generales
const float velocidadHoming = 50.0;
const float aceleracionHoming = 50.0;
const long limiteHomingPasos = 2100;
const int maxIntentosHoming = 3;
const int delayEntreIntentos = 2000;        // ms
const unsigned long timeoutHoming = 30000;  // 30 segundos

// Configuración de los tres motores (pines, velocidades, etc.)
StepperMotor motors[3] = {
  { AccelStepper(motorInterfaceType, 13, 14), 13, 14, 34, 0, 4000.0, 4000.0, 1 },
  { AccelStepper(motorInterfaceType, 22, 21), 22, 21, 36, -160, 1000.0, 2000.0, 2 },
  { AccelStepper(motorInterfaceType, 19, 18), 19, 18, 39, -40, 5000.0, 5000.0, 3 }
};

// Variables para control
bool homingCompletado = false;
bool homingIniciado = false;
unsigned long ultimoTiempo = 0;
const int intervaloChequeo = 500;  // Intervalo para verificar comandos BT (ms)

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Unificado");  // Nombre Bluetooth
  Serial.println("Robot inicializado. Esperando comandos Bluetooth.");
  Serial.println("Comandos disponibles:");
  Serial.println("  'H + wr(lo,hi) + wl(lo,hi)' - Para control de motores móviles");
  Serial.println("  'J INIT' - Para iniciar calibración de robot articulado");
  Serial.println("  'J M1 100, M1 -100, ...' - Para mover motores articulados");

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Inicializar pines de sensores para motores articulados
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].sensorPin, INPUT);
  }

  // Inicializar MCPWM para Motor A (izquierdo)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motorA2);

  // Configuración correcta de MCPWM
  mcpwm_config_t confA = {
    .frequency    = pwmFreq,              // 1º campo
    .cmpr_a       = 0,                    // 2º campo
    .cmpr_b       = 0,                    // 3º campo
    .duty_mode    = MCPWM_DUTY_MODE_0,    // 4º campo
    .counter_mode = MCPWM_UP_COUNTER      // 5º campo
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

// Función para realizar homing secuencial para los motores articulados
void realizarHomingSecuencial() {
  Serial.println("=== INICIANDO SECUENCIA DE HOMING ESTRICTAMENTE SECUENCIAL ===");
  SerialBT.println("=== INICIANDO SECUENCIA DE HOMING ===");

  // Configurar velocidades iniciales para homing
  for (int i = 0; i < 3; i++) {
    motors[i].stepper.setMaxSpeed(velocidadHoming);
    motors[i].stepper.setAcceleration(aceleracionHoming);
  }

  // Realizar homing de cada motor en secuencia
  for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
    bool homeExitoso = false;

    for (int intento = 1; intento <= maxIntentosHoming && !homeExitoso; intento++) {
      Serial.print("Motor ");
      Serial.print(motors[motorIndex].motorID);
      Serial.print(": Intento ");
      Serial.print(intento);
      Serial.println(" de homing");

      SerialBT.print("Motor ");
      SerialBT.print(motors[motorIndex].motorID);
      SerialBT.print(": Intento ");
      SerialBT.print(intento);
      SerialBT.println(" de homing");

      homeExitoso = homingMotorSeguro(motors[motorIndex]);

      if (!homeExitoso && intento < maxIntentosHoming) {
        Serial.print("Esperando antes del siguiente intento para Motor ");
        Serial.print(motors[motorIndex].motorID);
        Serial.println("...");
        delay(delayEntreIntentos);
      }
    }

    // Si el homing falló para este motor, detenerse completamente
    if (!homeExitoso) {
      Serial.print("ERROR CRÍTICO: Homing fallido en Motor ");
      Serial.print(motors[motorIndex].motorID);
      Serial.println(". No se puede continuar.");

      SerialBT.print("ERROR: Homing fallido en Motor ");
      SerialBT.print(motors[motorIndex].motorID);
      SerialBT.println(". Reinicia el sistema.");

      while (true) {
        delay(1000);  // Loop infinito de error
      }
    }

    Serial.print("Motor ");
    Serial.print(motors[motorIndex].motorID);
    Serial.println(": Homing exitoso.");

    SerialBT.print("Motor ");
    SerialBT.print(motors[motorIndex].motorID);
    SerialBT.println(": Homing exitoso.");

    if (motorIndex < 2) {
      delay(1000);  // Pausa antes del siguiente motor
    }
  }

  // Si llegamos aquí, todos los motores han hecho homing correctamente
  Serial.println("=== HOMING SECUENCIAL COMPLETADO EXITOSAMENTE EN LOS 3 MOTORES ===");
  SerialBT.println("=== HOMING COMPLETADO EN LOS 3 MOTORES ===");

  // Cambiar a velocidades normales para la rutina
  for (int i = 0; i < 3; i++) {
    motors[i].stepper.setMaxSpeed(motors[i].velocidadNormal);
    motors[i].stepper.setAcceleration(motors[i].aceleracionNormal);
    motors[i].stepper.setCurrentPosition(0);
  }

  homingCompletado = true;
  Serial.println("Sistema listo para comandos de movimiento.");
  SerialBT.println("Sistema listo para comandos de movimiento.");
}

bool homingMotorSeguro(StepperMotor &motor) {
  Serial.print("Homing motor ");
  Serial.print(motor.motorID);
  Serial.println(" hasta detectar sensor...");

  // Verificar si ya está en la posición de home
  if (digitalRead(motor.sensorPin) == LOW) {
    // Verificación doble para evitar falsas lecturas
    delay(50);
    if (digitalRead(motor.sensorPin) == LOW) {
      Serial.print("Motor ");
      Serial.print(motor.motorID);
      Serial.println(" ya está en posición de home!");
      motor.stepper.setCurrentPosition(0);
      return true;
    }
  }

  // Mover hacia atrás (negativo) para buscar el sensor
  motor.stepper.moveTo(-limiteHomingPasos);
  unsigned long startTime = millis();
  bool sensorDetectado = false;

  while (!sensorDetectado) {
    // Mover el motor un paso y verificar si debe detenerse
    motor.stepper.run();

    // Verificar si el sensor se activó (LOW)
    if (digitalRead(motor.sensorPin) == LOW) {
      // Verificación doble para evitar falsas lecturas
      delay(50);
      if (digitalRead(motor.sensorPin) == LOW) {
        // Detener el motor inmediatamente
        motor.stepper.stop();
        // Asegurarse de que el motor pare completamente
        while (motor.stepper.isRunning()) {
          motor.stepper.run();
        }
        Serial.print("Motor ");
        Serial.print(motor.motorID);
        Serial.println(": Sensor Hall detectado!");
        sensorDetectado = true;
      }
    }

    // Verificar si alcanzó el límite de pasos
    if (abs(motor.stepper.currentPosition()) >= limiteHomingPasos) {
      Serial.print("ERROR: Motor ");
      Serial.print(motor.motorID);
      Serial.println(" alcanzó el límite de pasos sin detectar sensor");
      return false;
    }

    // Verificar si se excedió el tiempo límite
    if (millis() - startTime > timeoutHoming) {
      Serial.print("ERROR: Motor ");
      Serial.print(motor.motorID);
      Serial.println(" superó el tiempo máximo de homing");
      return false;
    }
  }

  // Si llegó aquí, el homing fue exitoso
  motor.stepper.setCurrentPosition(0);
  Serial.print("Motor ");
  Serial.print(motor.motorID);
  Serial.println(": Homing completado con éxito");
  return true;
}

void moverMotorConPasos(StepperMotor &motor, int pasos) {
  int pasosAMover = (pasos == 0) ? motor.pasosPorVuelta : pasos;

  Serial.print("Moviendo motor ");
  Serial.print(motor.motorID);
  Serial.print(" con ");
  Serial.print(pasosAMover);
  Serial.println(" pasos");

  SerialBT.print("M");
  SerialBT.print(motor.motorID);
  SerialBT.print(": ");
  SerialBT.print(pasosAMover);
  SerialBT.println(" pasos");

  // Obtener la posición actual
  long posicionActual = motor.stepper.currentPosition();

  // Calcular posición objetivo (relativa a la posición actual)
  long posicionObjetivo = posicionActual + pasosAMover;

  // Mover hacia la posición objetivo (puede ser adelante o atrás)
  motor.stepper.moveTo(posicionObjetivo);

  while (motor.stepper.distanceToGo() != 0) {
    motor.stepper.run();
  }

  Serial.print("Motor ");
  Serial.print(motor.motorID);
  Serial.print(" ha completado su movimiento. Posición final: ");
  Serial.println(motor.stepper.currentPosition());
}

// Procesa un comando J para los motores articulados
void procesarComandoArticulados(String comando) {
  digitalWrite(ledPin, HIGH);  // Indicador LED
  
  // Remover el "J " inicial
  comando = comando.substring(2);
  comando.trim();
  
  if (comando == "INIT") {
    // Iniciar secuencia de homing si no se ha hecho
    if (!homingCompletado) {
      homingIniciado = true;
      realizarHomingSecuencial();
    } else {
      Serial.println("El homing ya está completado.");
      SerialBT.println("El homing ya está completado.");
    }
  } else {
    // Verificar si el homing está completado
    if (!homingCompletado) {
      SerialBT.println("ERROR: Debe inicializar primero con 'J INIT'");
      Serial.println("ERROR: Debe inicializar primero con 'J INIT'");
      digitalWrite(ledPin, LOW);
      return;
    }
    
    // Separar comandos por comas
    int startPos = 0;
    int commaPos;
    
    do {
      commaPos = comando.indexOf(',', startPos);
      String subComando;
      
      if (commaPos != -1) {
        subComando = comando.substring(startPos, commaPos);
        startPos = commaPos + 1;
      } else {
        subComando = comando.substring(startPos);
      }
      
      subComando.trim();
      
      // Formato esperado: "M1 100" o similar
      if (subComando.startsWith("M")) {
        int espacio = subComando.indexOf(' ');
        if (espacio > 0) {
          int motorNum = subComando.substring(1, espacio).toInt();
          int pasos = subComando.substring(espacio + 1).toInt();
          
          if (motorNum >= 1 && motorNum <= 3) {
            // Mover el motor especificado
            moverMotorConPasos(motors[motorNum-1], pasos);
            // Esperar 200ms entre movimientos como solicitado
            delay(200);
          } else {
            SerialBT.print("Motor inválido: ");
            SerialBT.println(motorNum);
          }
        }
      }
    } while (commaPos != -1);
  }
  
  digitalWrite(ledPin, LOW);  // Apagar indicador LED
}

// Procesa un comando H para los motores móviles
void procesarComandoMovil() {
  Serial.println("Entrando a procesarComandoMovil");
  
  // Leer el comando completo
  String comando = "";
  unsigned long startTime = millis();
  
  // Leer hasta encontrar salto de línea o timeout
  while ((millis() - startTime) < 2000) {  // 2 segundos de timeout
    if (SerialBT.available()) {
      char c = SerialBT.read();
      comando += c;
      Serial.print(c);  // Debug - mostrar cada carácter
      if (c == '\n' || c == '\r') {
        break;  // Terminar cuando se encuentre un fin de línea
      }
    }
    yield();  // Permitir procesos de fondo de ESP32
  }
  
  Serial.print("Comando completo: ");
  Serial.println(comando);
  
  comando.trim();  // Eliminar espacios y saltos de línea
  
  // Dividir la cadena para obtener las velocidades
  // Esperamos formato: "100 100" (ya se leyó el 'H' en loop())
  int espacio = comando.indexOf(' ');
  if (espacio <= 0) {
    // Si no hay espacio, podría ser solo un número
    Serial.println("Error: formato inválido, falta espacio separador");
    return;
  }
  
  String wrStr = comando.substring(0, espacio);
  String wlStr = comando.substring(espacio + 1);
  
  Serial.print("wrStr: ");
  Serial.println(wrStr);
  Serial.print("wlStr: ");
  Serial.println(wlStr);
  
  int16_t wr = wrStr.toInt();
  int16_t wl = wlStr.toInt();
  
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
  setMotorB(dutyWr);  // Motor derecho
  setMotorA(dutyWl);  // Motor izquierdo
  
  // PRINT en serial para debug
  Serial.print("wr: ");
  Serial.print(wr);
  Serial.print(" wl: ");
  Serial.print(wl);
  Serial.print(" dutyWr: ");
  Serial.print(dutyWr);
  Serial.print(" dutyWl: ");
  Serial.print(dutyWl);
  Serial.println();
  
  // Apagar indicador después de un breve delay
  delay(10);
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Procesar comandos de homing si se solicitó y aún no está completado
  if (homingIniciado && !homingCompletado) {
    realizarHomingSecuencial();
    homingIniciado = false;  // Resetear la bandera
  }
  
  // Comprobar si hay datos disponibles por Bluetooth
  if (SerialBT.available() > 0) {
    char commandType = SerialBT.read();  // Leer primer carácter
    
    // Imprimir para debug qué comando se recibió
    Serial.print("Comando recibido: ");
    Serial.println(commandType);
    
    // Comando H para motores móviles
    if (commandType == 'H') {
      Serial.println("Procesando comando de motores móviles...");
      procesarComandoMovil();
    } 
    // Comando J para motores articulados
    else if (commandType == 'J') {
      Serial.println("Procesando comando de motores articulados...");
      // Esperar a que llegue el comando completo
      String comando = "J ";
      unsigned long startTime = millis();
      
      // Leer hasta encontrar salto de línea o timeout
      while ((millis() - startTime) < 5000) {  // 5 segundos de timeout
        if (SerialBT.available()) {
          char c = SerialBT.read();
          comando += c;
          if (c == '\n' || c == '\r') {
            break;  // Terminar cuando se encuentre un fin de línea
          }
        }
        yield();  // Permitir procesos de fondo de ESP32
      }
      
      comando.trim();  // Eliminar espacios y saltos de línea
      
      if (comando.length() > 2) {  // Si hay comando válido (más que solo "J ")
        procesarComandoArticulados(comando);
      }
    } 
    // Comando no reconocido - descartar
    else {
      Serial.print("Comando no reconocido: ");
      Serial.println(commandType);
      // Limpiar buffer
      while (SerialBT.available()) {
        SerialBT.read();
      }
    }
  }
  
  // Ejecutar los pasos necesarios para los motores stepper
  for (int i = 0; i < 3; i++) {
    motors[i].stepper.run();
  }
}