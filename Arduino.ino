#include <Servo.h>

// Define servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

// Motor Pins
int ENA = 12;
int ENB = 7;
int MOTOR_A1 = 11;
int MOTOR_A2 = 10;
int MOTOR_B1 = 9;
int MOTOR_B2 = 8;

// IR Sensor Pins
int IR_RIGHT = A0;
int IR_LEFT = A1;

bool isStopped = false;

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Attach servos to pins
  baseServo.attach(2);
  shoulderServo.attach(4);
  elbowServo.attach(5);
  wristServo.attach(6);

  // Initialize motor and sensor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);

  Serial.println("4-DOF Robot Arm Ready.");
}

void loop() {
  int rightSensor = digitalRead(IR_RIGHT);
  int leftSensor = digitalRead(IR_LEFT);

  if (rightSensor == HIGH && leftSensor == HIGH) {
    if (!isStopped) {
      stopMotors();
      Serial.println("Obstacle detected! Waiting for angle commands...");
      isStopped = true;
    }
    handleSerialCommunication();  // Continuously check for angle commands
  } else {
    if (isStopped) {
      Serial.println("Moving again...");
      isStopped = false;
    }

    if (rightSensor == LOW && leftSensor == LOW) {
      moveForward();
    } else if (rightSensor == LOW && leftSensor == HIGH) {
      turnLeft();
    } else if (rightSensor == HIGH && leftSensor == LOW) {
      turnRight();
    }
  }

  delay(100);
}

void handleSerialCommunication() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int baseAngle, shoulderAngle, elbowAngle, wristAngle;
    sscanf(input.c_str(), "%d,%d,%d,%d", &baseAngle, &shoulderAngle, &elbowAngle, &wristAngle);

    baseServo.write(constrain(baseAngle, 0, 180));
    shoulderServo.write(constrain(shoulderAngle, 0, 180));
    elbowServo.write(constrain(elbowAngle, 0, 180));
    wristServo.write(constrain(wristAngle, 0, 180));

    Serial.println("Angles received and servos moved.");
  }
}

void moveForward() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void turnLeft() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 0);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
}

void turnRight() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 80);
  digitalWrite(MOTOR_B2, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
