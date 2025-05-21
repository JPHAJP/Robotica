#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "BluetoothSerial.h"
#include <AccelStepper.h>

//Banderas
bool funcionHoming = false; // bandera para controlar Homing
bool yaEnderezado = false;  // bandera para controlar Enderezado

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
const int pasosHoming = 50;
// Parámetros exclusivos para homing
const float velocidadHoming   = 50.0;   // pasos por segundo en homing
const float aceleracionHoming = 150.0;   // pasos/s² en homing

// Configuración de los tres motores (pines, velocidades, etc.)
StepperMotor motors[3] = {
  { AccelStepper(motorInterfaceType, 13, 14), 13, 14, 34, 0, 4000.0, 4000.0, 1 },
  { AccelStepper(motorInterfaceType, 22, 21), 22, 21, 36, -160, 1000.0, 2000.0, 2 },
  { AccelStepper(motorInterfaceType, 19, 18), 19, 18, 39, -40, 5000.0, 5000.0, 3 }
};

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
////////////////////////////////////////////////////////
bool Homing() {
  unsigned long tiempoInicio = millis(); // Momento de inicio
  const unsigned long tiempoLimite = 13000; // 6 segundos en milisegundos
  while (true) {
    // Verificar si han pasado 10 segundos
    if (millis() - tiempoInicio >= tiempoLimite) {
      // Detener todos los motores
      for (int i = 0; i < 3; i++) {
        motors[i].stepper.stop();
      }
      break;
    }
    for (int i = 0; i < 3; i++) {
    AccelStepper &st      = motors[i].stepper;
    int pinSensor = motors[i].sensorPin;
    
    // Si ya terminó su movimiento anterior, iniciamos homing
    if (st.distanceToGo() == 0) {
        // Aplicar parámetros de homing
        st.setMaxSpeed(velocidadHoming);
        st.setAcceleration(aceleracionHoming);
        
        // Motor 1 (i=1) dirección positiva, motores 0 y 2 dirección negativa
        int pasos = (i == 1) ? pasosHoming : -pasosHoming;
        st.move(pasos);
    }
    
    // Lectura del sensor Hall de este motor
    if (digitalRead(pinSensor) == LOW) {
        st.stop();  // detiene el motor con aceleración
    } else {
        st.run();   // continúa el movimiento
    }
}
  }
  return true; // Función completada
}

void enderezar() {
  
  // Motor 2 (índice 1): -180 pasos
  {
    AccelStepper &st = motors[1].stepper;
    st.setMaxSpeed(velocidadHoming);
    st.setAcceleration(aceleracionHoming);
    st.moveTo(st.currentPosition() - 175);
    st.runToPosition();
  }
  // Motor 3 (índice 2): +180 pasos
  {
    AccelStepper &st = motors[2].stepper;
    st.setMaxSpeed(velocidadHoming);
    st.setAcceleration(aceleracionHoming);
    st.moveTo(st.currentPosition() + 175);
    st.runToPosition();
  }
}
///////////////////////////////////////////////////////

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

void procesarComandoArticulados(String comando) {
  digitalWrite(ledPin, HIGH);  // Indicador LED
  
  // Remover el "J " inicial
  comando = comando.substring(2);
  comando.trim();
  
  if (comando == "INIT") {
    // Ejecutar homing solo una vez
    if (!funcionHoming) {
        Homing();
        funcionHoming = true;
    }
    // Ejecutar homing solo una vez
    if (!yaEnderezado) {
        enderezar();
        yaEnderezado = true;
    }
  }
    // Dividir la trayectoria completa en movimientos individuales
    int movStartPos = 0;
    int movEndPos;
    
    do {
      movEndPos = comando.indexOf('|', movStartPos);
      String movimiento;
      
      if (movEndPos != -1) {
        movimiento = comando.substring(movStartPos, movEndPos);
        movStartPos = movEndPos + 1;
      } else {
        movimiento = comando.substring(movStartPos);
      }
      
      movimiento.trim();
      ejecutarMovimientoSimultaneo(movimiento);
      
    } while (movEndPos != -1);
  
  
  digitalWrite(ledPin, LOW);  // Apagar indicador LED
}

void ejecutarMovimientoSimultaneo(String movimiento) {
  // Arrays para almacenar la configuración de cada motor
  int motorIds[3] = {0, 0, 0};
  int pasos[3] = {0, 0, 0};
  float velocidades[3] = {0, 0, 0};
  float aceleraciones[3] = {0, 0, 0};
  int motorCount = 0;
  
  // Dividir el movimiento en configuraciones para cada motor
  int segStartPos = 0;
  int segEndPos;
  
  do {
    segEndPos = movimiento.indexOf(';', segStartPos);
    String configuracion;
    
    if (segEndPos != -1) {
      configuracion = movimiento.substring(segStartPos, segEndPos);
      segStartPos = segEndPos + 1;
    } else {
      configuracion = movimiento.substring(segStartPos);
    }
    
    configuracion.trim();
    
    // Parsear la configuración del motor (id,pasos,velocidad,aceleracion)
    int paramsPos = 0;
    int commaPos;
    int id = -1, steps = 0;
    float vel = 0, accel = 0;
    
    commaPos = configuracion.indexOf(',');
    if (commaPos > 0) {
      id = configuracion.substring(0, commaPos).toInt();
      paramsPos = commaPos + 1;
    }
    
    commaPos = configuracion.indexOf(',', paramsPos);
    if (commaPos > 0) {
      steps = configuracion.substring(paramsPos, commaPos).toInt();
      paramsPos = commaPos + 1;
    }
    
    commaPos = configuracion.indexOf(',', paramsPos);
    if (commaPos > 0) {
      vel = configuracion.substring(paramsPos, commaPos).toFloat();
      paramsPos = commaPos + 1;
      accel = configuracion.substring(paramsPos).toFloat();
    }
    
    // Si el ID es válido y tenemos espacio en el array
    if (id >= 1 && id <= 3 && motorCount < 3) {
      int index = id - 1;  // Convertir ID (1-3) a índice (0-2)
      motorIds[motorCount] = index;
      pasos[motorCount] = steps;
      velocidades[motorCount] = vel;
      aceleraciones[motorCount] = accel;
      motorCount++;
      
      Serial.print("Motor ");
      Serial.print(id);
      Serial.print(": Pasos=");
      Serial.print(steps);
      Serial.print(", Vel=");
      Serial.print(vel);
      Serial.print(", Accel=");
      Serial.println(accel);
    }
    
  } while (segEndPos != -1 && motorCount < 3);
  
  // Configurar y mover todos los motores simultáneamente
  if (motorCount > 0) {
    // Configurar los motores
    for (int i = 0; i < motorCount; i++) {
      int motorIndex = motorIds[i];
      motors[motorIndex].stepper.setMaxSpeed(velocidades[i]);
      motors[motorIndex].stepper.setAcceleration(aceleraciones[i]);
      
      // Obtener posición actual y calcular posición objetivo
      long posicionActual = motors[motorIndex].stepper.currentPosition();
      long posicionObjetivo = posicionActual + pasos[i];
      motors[motorIndex].stepper.moveTo(posicionObjetivo);
    }
    
    SerialBT.println("Ejecutando movimiento simultáneo");
    
    // Ejecutar el movimiento hasta que todos los motores alcancen su posición
    bool movimientoCompleto = false;
    while (!movimientoCompleto) {
      movimientoCompleto = true;
      
      // Mover cada motor un paso si es necesario
      for (int i = 0; i < motorCount; i++) {
        int motorIndex = motorIds[i];
        if (motors[motorIndex].stepper.distanceToGo() != 0) {
          motors[motorIndex].stepper.run();
          movimientoCompleto = false;
        }
      }
      
      // Si no se configuró ningún motor activo, salir del bucle
      if (motorCount == 0) {
        movimientoCompleto = true;
      }
    }
    
    SerialBT.println("Movimiento completado");
  }
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