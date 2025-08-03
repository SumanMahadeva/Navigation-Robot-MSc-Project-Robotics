#include <Arduino.h>
#include <ARB.h>
#include <Wire.h>
#include <mbed.h>

// Define constants for IR sensor and motor control pins
#define IR_SENSOR_ADDRESS      0x80 >> 1
#define IR_SENSOR_DISTANCE_REG 0x5E
#define IR_SENSOR_SHIFT_REG    0x35

#define irSensorPin A0
#define motor1Pin1 6
#define motor1Pin2 7
#define motor2Pin1 8
#define motor2Pin2 9

// Enumeration for motor direction and selection
typedef enum {CW, CCW} Direction;
typedef enum {A, B} Motor;

// Volatile variables to store step count from motors
volatile int stepsA = 0;
volatile int stepsB = 0;

// Arrays to store ultrasonic sensor readings
int duration[2], cm[2];
int ccm1;
int ccm2;

// Function prototypes
int readIRSensorDistance();
int ultra();
int ulra1();
void moveForward();
void turnLeft();
void turnLeft1();
void turnRight1();
void turnRight();
void stopRobot();

void setup() {
  Serial.begin(115200);
  ARBSetup(true);

  // Setup pins for motors
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);

  Wire.begin();
}

void loop() {
  int irDistance = readIRSensorDistance();
  ultra();
  ulra1();

  // Print sensor values
  Serial.print("IR Sensor Value: ");
  Serial.println(irDistance);
  Serial.print("Ultrasonic - Left Distance: ");
  Serial.print(ccm1);
  Serial.print(" CM, Right Distance: ");
  Serial.println(ccm2);

  // Send sensor data to Raspberry Pi
  putRegister(0, int(irDistance));
  putRegister(1, int(ccm1));
  putRegister(2, int(ccm2));

  // Receive control signal from Raspberry Pi
  char number = getRegister(3);

  // Act based on the received control signal
  switch (number) {
    case 'K':
      if (irDistance < 12) {
        if(ccm2 > ccm1) {
          turnRight();
          Serial.print(" Turn Right ");
        } else {
          turnLeft();
          Serial.print(" Turn Left ");
        }
      } else if (ccm1 < 10) {
          turnRight1();
          Serial.print(" Turn Right 30 degree ");
          delay(500);
      } else if (ccm2 < 10) {
          turnLeft1();
          Serial.print(" Turn Left 30 degree ");
          delay(500);
      } else {
        moveForward();
        Serial.print(" Move Forward ");
      }
      break;
      case 'R':
        //turn the robot Right
        turnRight();
        break;

      case 'F':
        //the robot MoveForward 
        moveForward();
        break;  
      
      case 'L':
        //turn the robot Left
        turnLeft();
        break;
      case 'U':
        //the robot Stop
        stopRobot();
        break;
    }
  

  // Update serial communication
  serialUpdate();
}

// Function to read distance from IR sensor
int readIRSensorDistance() {
  int distance = 0;
  byte high, low;

  setI2CBus(0);

  // Request shift value from IR sensor
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_SHIFT_REG);
  Wire.endTransmission();

  // Wait until data is available
  Wire.requestFrom(IR_SENSOR_ADDRESS, 1);
  while (Wire.available() == 0) {}

  int shift = Wire.read();

  // Request distance data from IR sensor
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_DISTANCE_REG);
  Wire.endTransmission();

  // Wait until data is available
  Wire.requestFrom(IR_SENSOR_ADDRESS, 2);
  while (Wire.available() < 2) {}

  // Read high and low bytes
  high = Wire.read();
  low = Wire.read();

  // Calculate the distance in cm
  distance = (high * 16 + low) / 16 / (int)pow(2, shift);
  return distance;
}

// Function to perform ultrasonic measurement on sensor 1
int ultra() {
  // Set trigger pulse for ultrasonic sensor 1
  pinMode(USONIC1, OUTPUT);
  digitalWrite(USONIC1, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC1, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC1, LOW);

  // Set the pin to read the returning signal
  pinMode(USONIC1, INPUT);
  duration[0] = pulseIn(USONIC1, HIGH);
  cm[0] = uSecToCM(duration[0]);

  // Print and update global variable
  Serial.print("Distance 1: ");
  Serial.print(cm[0]);
  Serial.println(" cm.");
  ccm1 = cm[0];
}

// Function to perform ultrasonic measurement on sensor 2
int ulra1() {
  // Set trigger pulse for ultrasonic sensor 2
  pinMode(USONIC2, INPUT);
  pinMode(USONIC2, OUTPUT);
  digitalWrite(USONIC2, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC2, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC2, LOW);

  // Read duration and convert to cm
  duration[1] = pulseIn(USONIC2, HIGH);
  cm[1] = uSecToCM(duration[1]);

  // Print and update global variable
  Serial.print("Distance 2: ");
  Serial.print(cm[1]);
  Serial.println(" cm.");
  ccm2 = cm[1];
}

// Function to move the robot forward
void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(MOTOR_DIRA, CW);
  analogWrite(MOTOR_PWMA, 140);
  digitalWrite(MOTOR_DIRB, CCW);
  analogWrite(MOTOR_PWMB, 140);
}

// Function to turn the robot left
void turnLeft() {
  for (int i = 0; i < 1; i++) {
    Serial.println("Moving left");
    digitalWrite(MOTOR_DIRA, CW);
    digitalWrite(MOTOR_DIRB, CW);
    analogWrite(MOTOR_PWMA, 115);
    analogWrite(MOTOR_PWMB, 80);
  }
  delay(1900);
}

// Function to turn the robot 25 degrees left
void turnLeft1() {
  for (int i = 0; i < 1; i++) {
    Serial.println("Moving left");
    digitalWrite(MOTOR_DIRA, CW);
    digitalWrite(MOTOR_DIRB, CW);
    analogWrite(MOTOR_PWMA, 70);
    analogWrite(MOTOR_PWMB, 0);
  }
}

// Function to turn the robot 25 degrees right
void turnRight1() {
  for (int i = 0; i < 1; i++) {
    Serial.println("Moving right");
    digitalWrite(MOTOR_DIRA, CCW);
    digitalWrite(MOTOR_DIRB, CCW);
    analogWrite(MOTOR_PWMA, 0);
    analogWrite(MOTOR_PWMB, 70);
  }
}

// Function to turn the robot right
void turnRight() {
  for (int i = 0; i < 1; i++) {
    Serial.println("Moving right");
    digitalWrite(MOTOR_DIRA, CCW);
    digitalWrite(MOTOR_DIRB, CCW);
    analogWrite(MOTOR_PWMA, 115);
    analogWrite(MOTOR_PWMB, 80);
  }
  delay(1900);
}

// Function to stop the robot
void stopRobot() {
  Serial.println("Stopping");
  digitalWrite(MOTOR_DIRA, LOW);
  digitalWrite(MOTOR_DIRB, LOW);
  analogWrite(MOTOR_PWMA, 0);
  analogWrite(MOTOR_PWMB, 0);
}
// Function to turn 180 degree the robot
void turn180Degrees()
{
  Serial.println("Turning 180 degrees");

  digitalWrite(MOTOR_DIRA, CW);
  digitalWrite(MOTOR_DIRB, CW);
  analogWrite(MOTOR_PWMA, 100);
  analogWrite(MOTOR_PWMB, 100);

  delay(500);  
  // Stop the motors
  digitalWrite(MOTOR_PWMA, LOW);
  digitalWrite(MOTOR_PWMB, LOW);
  
  delay(1000);  
}
