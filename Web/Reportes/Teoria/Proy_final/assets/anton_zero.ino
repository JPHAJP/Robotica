#include <AccelStepper.h>
bool funcionHoming = false;
// motorInterfaceType = 1 para Step + Dir
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
 
// Configuración de los tres motores (pines, velocidades, etc.)
StepperMotor motors[3] = {
  { AccelStepper(motorInterfaceType, 13, 14), 13, 14, 34, 0,    4000.0, 4000.0, 1 },
  { AccelStepper(motorInterfaceType, 22, 21), 22, 21, 36, -160, 1000.0, 2000.0, 2 },
  { AccelStepper(motorInterfaceType, 19, 18), 19, 18, 39, -40,  5000.0, 5000.0, 3 }
};
 
const int pasosHoming = 50;
 
// Parámetros exclusivos para homing
const float velocidadHoming   = 500.0;   // pasos por segundo en homing
const float aceleracionHoming = 500.0;   // pasos/s² en homing
 
void setup() {
  Serial.begin(115200);
 
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].sensorPin, INPUT);
    // Configurar velocidad y aceleración "normales"
    motors[i].stepper.setMaxSpeed(motors[i].velocidadNormal);
    motors[i].stepper.setAcceleration(motors[i].aceleracionNormal);
    Serial.print("Motor ");
    Serial.print(motors[i].motorID);
    Serial.println(" listo");
  }
}
 
bool Homing() {
  for (int i = 0; i < 3; i++) {
    AccelStepper &st      = motors[i].stepper;
    int          pinSensor = motors[i].sensorPin;
 
    // Si ya terminó su movimiento anterior, iniciamos homing
    if (st.distanceToGo() == 0) {
      // Aplicar parámetros de homing
      st.setMaxSpeed(velocidadHoming);
      st.setAcceleration(aceleracionHoming);
 
      // Tercer motor gira en sentido contrario
      int pasos = (i == 2) ? -pasosHoming : pasosHoming;
      st.move(pasos);
 
      Serial.print("Motor ");
      Serial.print(motors[i].motorID);
      Serial.print(": homing de ");
      Serial.print(abs(pasos));
      Serial.print(" pasos");
      if (i == 2) Serial.print(" (sentido inverso)");
      Serial.println();
    }
 
    // Lectura del sensor Hall de este motor
    if (digitalRead(pinSensor) == LOW) {
      st.stop();  // detiene el motor con aceleración
      Serial.print("Motor ");
      Serial.print(motors[i].motorID);
      Serial.println(" - Sensor activado, deteniendo.");
    } else {
      st.run();   // continúa el movimiento
    }
  }
}
 
 
 
void loop(){
  Homing();
}