#include <BluetoothSerial.h>
#include <AccelStepper.h>

const int num_q=3;
//Declaracion de objetos de Motor a Pasos
AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN_MOTOR1, DIR_PIN_MOTOR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN_MOTOR2, DIR_PIN_MOTOR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP_PIN_MOTOR3, DIR_PIN_MOTOR3);

//Declaracion de Objeto de Bluetooth
BluetoothSerial SerialBT;


void calibrar(AccelStepper& stepper, bool dir, int SwitchPin);
void mover_kin_rot(AccelStepper& stepper, float angle, float relacion, float speed);
int rpmtosteps(float rpm);

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  Serial.println("Starting");
  delay(1000);
  
  
  SerialBT.begin("Robot");
  /********Configuracion de Final de Carrera*********/
}

void loop() {
  receive();
}


void receive() {
  char buff[128] = "";
  String cmd = "";

  if (SerialBT.available() > 0) {
    cmd = SerialBT.readStringUntil('\n');
    Serial.print("MENSAJE: ");
    Serial.println(cmd);

    int segmentIndex = 0;
    int prevIndex = 0;
    int nextIndex = cmd.indexOf(';');

    while (nextIndex >= 0) {
      String segment = cmd.substring(prevIndex, nextIndex);
      processSegment(segment, segmentIndex);
      segmentIndex++;
      prevIndex = nextIndex + 1;
      nextIndex = cmd.indexOf(';', prevIndex);
    }

    // Último segmento (o único si no hay ';')
    if (prevIndex < cmd.length()) {
      processSegment(cmd.substring(prevIndex), segmentIndex);
    }
  }
}

void processSegment(String segment, int index) {
  Serial.printf("Segmento #%d: %s\n", index, segment.c_str());
  
  // Arrays to store positions and speeds for the three joints.
  int positions[num_q];
  int speeds[num_q];
  int jointIndex = 0;
  int prev = 0;
  int next = segment.indexOf(',');

  // Process each comma-separated pair.
  while (next >= 0 && jointIndex < num_q) {
    String pair = segment.substring(prev, next);
    if (!parseAndStorePair(pair, jointIndex, positions, speeds)) {
      Serial.printf("Error al parsear la pareja en el joint %d\n", jointIndex);
      return;  // Stop processing if an error occurs.
    }
    jointIndex++;
    prev = next + 1;
    next = segment.indexOf(',', prev);
  }

  // Process the last pair (or the only pair if there were no commas)
  if (jointIndex < num_q && prev < segment.length()) {
    String pair = segment.substring(prev);
    if (!parseAndStorePair(pair, jointIndex, positions, speeds)) {
      Serial.printf("Error al parsear la pareja en el joint %d\n", jointIndex);
      return;
    }
    jointIndex++;
  }

  // Check if we got all the joint data
  if (jointIndex == num_q) {
    // Call the move function so that all motors start simultaneously.
    move(positions[0], positions[1], positions[2],
         speeds[0], speeds[1], speeds[2]);
  } else {
    Serial.println("Error: Datos insuficientes de joints en el segmento");
  }
}

// This helper function parses a "pos@speed" pair and stores the results in the arrays.
bool parseAndStorePair(String pair, int jointIndex, int positions[], int speeds[]) {
  int atIndex = pair.indexOf('@');
  if (atIndex > 0) {
    int pos = pair.substring(0, atIndex).toInt();
    int speed = pair.substring(atIndex + 1).toInt();
    positions[jointIndex] = pos;
    speeds[jointIndex] = speed;
    Serial.printf("J%d -> Pos: %d, Speed: %d pasos/s\n", jointIndex + 1, pos, speed);
    return true;
  } else {
    Serial.printf("Error: no se encontró '@' en %s\n", pair.c_str());
    return false;
  }
}

void move(int pos1, int pos2, int pos3, int speed1, int speed2, int speed3) {
  // Set speeds and target positions for each motor.
  
  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);
  motor3.setSpeed(speed3);
  motor1.moveTo(pos1);
  motor2.moveTo(pos2);
  motor3.moveTo(pos3);
  Serial.println("Moving");
  // Run motors until all reach their target.
  while (motor1.distanceToGo()>0 || motor2.distanceToGo()>0 || motor3.distanceToGo()>0) {
    motor1.runSpeedToPosition();
    //checkMotor(motor1, 1);
    motor2.runSpeedToPosition();
    //checkMotor(motor2, 2);
    motor3.runSpeedToPosition();
    //checkMotor(motor3, 3);
  }
}

void checkMotor(AccelStepper& stepper, int num){
  
    Serial.print(num);
    Serial.print(": ");
    Serial.println(stepper.distanceToGo());

}
/

