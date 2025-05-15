#include <AccelStepper.h>

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

#define motorInterfaceType 1

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

void setup() {
  Serial.begin(250000);
  // ---- Esperar comando para iniciar calibración ----
  Serial.println("Presione ENTER para iniciar la calibración...");
  while (Serial.read() != '\n') {
    // espera hasta recibir un salto de línea
  }
  Serial.println("Iniciando calibración...");
  // ---------------------------------------------------

  // Configurar pines de sensores y velocidades iniciales para homing
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].sensorPin, INPUT);
    motors[i].stepper.setMaxSpeed(velocidadHoming);
    motors[i].stepper.setAcceleration(aceleracionHoming);
  }

  Serial.println("Iniciando secuencia de homing estrictamente secuencial...");
  Serial.println("Una vez completado, puede enviar comandos por Serial:");
  Serial.println("  'MX Y' donde X = número de motor (1-3, o 0 para todos)");
  Serial.println("  Y = número de pasos (0 para usar valor por defecto)");
  Serial.println("Ejemplo: 'M1 200' mueve el motor 1 con 200 pasos");

  realizarHomingSecuencial();

  // Inicializar la variable de control para el loop
  //motorAMover = -99; // Valor que representa "sin acción pendiente"
}

void realizarHomingSecuencial() {
  Serial.println("=== INICIANDO SECUENCIA DE HOMING ESTRICTAMENTE SECUENCIAL ===");

  // Realizar homing de cada motor en secuencia
  for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
    bool homeExitoso = false;

    for (int intento = 1; intento <= maxIntentosHoming && !homeExitoso; intento++) {
      Serial.print("Motor ");
      Serial.print(motors[motorIndex].motorID);
      Serial.print(": Intento ");
      Serial.print(intento);
      Serial.println(" de homing");

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
      Serial.print("El sistema requiere que el Motor ");
      Serial.print(motors[motorIndex].motorID);
      Serial.println(" encuentre su origen.");

      while (true) {
        delay(1000);  // Loop infinito de error
      }
    }

    Serial.print("Motor ");
    Serial.print(motors[motorIndex].motorID);
    Serial.println(": Homing exitoso.");

    if (motorIndex < 2) {
      Serial.print("Pasando al Motor ");
      Serial.print(motors[motorIndex + 1].motorID);
      Serial.println(".");
      delay(1000);  // Pausa antes del siguiente motor
    }
  }

  // Si llegamos aquí, todos los motores han hecho homing correctamente
  Serial.println("=== HOMING SECUENCIAL COMPLETADO EXITOSAMENTE EN LOS 3 MOTORES ===");
  Serial.println("Configurando velocidades normales para rutina de movimiento...");

  // Cambiar a velocidades normales para la rutina
  for (int i = 0; i < 3; i++) {
    motors[i].stepper.setMaxSpeed(motors[i].velocidadNormal);
    motors[i].stepper.setAcceleration(motors[i].aceleracionNormal);
    motors[i].stepper.setCurrentPosition(0);
  }

  homingCompletado = true;
  Serial.println("Sistema listo. Comenzando rutina normal.");
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

void moverMotor(StepperMotor &motor) {
  moverMotorConPasos(motor, 0);  // Usar pasosPorVuelta por defecto
}

// Variables para control de los motores en loop
int motorAMover = -1;         // -1 = todos los motores, 0,1,2 = motor específico
int pasosPersonalizados = 0;  // Cantidad de pasos a mover (0 = usar pasosPorVuelta por defecto)
unsigned long ultimoTiempo = 0;
const int intervaloChequeo = 500;  // Intervalo para verificar comandos seriales (ms)

void procesarComandoSerial() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    // Formato: "MX Y"
    // Donde X = número de motor (1-3 o 0 para todos)
    // Y = número de pasos (puede ser positivo o negativo)

    if (comando.startsWith("M")) {
      int espacio = comando.indexOf(' ');
      if (espacio > 0) {
        int motor = comando.substring(1, espacio).toInt();
        int pasos = comando.substring(espacio + 1).toInt();  // Este método maneja correctamente valores negativos

        // Validar motor (0=todos, 1-3=motor específico)
        if (motor >= 0 && motor <= 3) {
          motorAMover = motor - 1;  // Convertir a índice de array (0-2) o -1 para todos
          pasosPersonalizados = pasos;

          Serial.print("Comando recibido: Motor ");
          if (motorAMover == -1) {
            Serial.print("TODOS");
          } else {
            Serial.print(motorAMover + 1);
          }
          Serial.print(", Pasos: ");
          if (pasosPersonalizados == 0) {
            Serial.println("por defecto");
          } else {
            Serial.println(pasosPersonalizados);
          }
          return;  // Comando procesado, salir
        }
      }
    }

    // Si llegamos aquí, el comando no fue reconocido
    Serial.println("Comando no reconocido. Use formato: 'MX Y'");
    Serial.println("X = número de motor (1-3, o 0 para todos)");
    Serial.println("Y = número de pasos (positivo o negativo, 0 para usar valor por defecto)");
  }
}

void moverMotorConPasos(StepperMotor &motor, int pasos) {
  int pasosAMover = (pasos == 0) ? motor.pasosPorVuelta : pasos;

  Serial.print("Moviendo motor ");
  Serial.print(motor.motorID);
  Serial.print(" con ");
  Serial.print(pasosAMover);
  Serial.println(" pasos");

  // Obtener la posición actual
  long posicionActual = motor.stepper.currentPosition();

  // Calcular posición objetivo (relativa a la posición actual)
  long posicionObjetivo = posicionActual + pasosAMover;

  Serial.print("Posición actual: ");
  Serial.print(posicionActual);
  Serial.print(", Posición objetivo: ");
  Serial.println(posicionObjetivo);

  // Mover hacia la posición objetivo (puede ser adelante o atrás)
  motor.stepper.moveTo(posicionObjetivo);

  while (motor.stepper.distanceToGo() != 0) {
    motor.stepper.run();

    // Mostrar velocidad cada 100 pasos para no saturar el serial
    if (motor.stepper.currentPosition() % 100 == 0) {
      Serial.print("M");
      Serial.print(motor.motorID);
      Serial.print(" Pos:");
      Serial.print(motor.stepper.currentPosition());
      Serial.print(" SPEED:");
      Serial.println(motor.stepper.speed());
    }
  }

  Serial.print("Motor ");
  Serial.print(motor.motorID);
  Serial.print(" ha completado su movimiento. Posición final: ");
  Serial.println(motor.stepper.currentPosition());
  delay(500);
}

void loop() {
  // Verificar si hay comandos en el puerto serial
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoTiempo >= intervaloChequeo) {
    procesarComandoSerial();
    ultimoTiempo = tiempoActual;
  }

  // Solo ejecutar la rutina si el homing se completó con éxito
  if (homingCompletado) {
    // Verificar si hay un comando de movimiento pendiente
    if (motorAMover != -99) {  // -99 es un valor que nunca se asigna, usado como bandera "sin acción"
      if (motorAMover == -1) {
        // Mover todos los motores secuencialmente
        for (int i = 0; i < 3; i++) {
          moverMotorConPasos(motors[i], pasosPersonalizados);
        }
      } else if (motorAMover >= 0 && motorAMover < 3) {
        // Mover un motor específico
        moverMotorConPasos(motors[motorAMover], pasosPersonalizados);
      }

      // Resetear para esperar nuevo comando
      motorAMover = -99;
      pasosPersonalizados = 0;
    }
  } else {
    // Si de alguna forma llegamos aquí sin homing completado, reintentar
    Serial.println("ADVERTENCIA: Intentando realizar homing nuevamente");
    realizarHomingSecuencial();
    delay(2000);
  }
}