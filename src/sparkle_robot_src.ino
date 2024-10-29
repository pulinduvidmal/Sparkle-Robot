#include <L298N.h>
#include <Arduino.h>
#include <Servo.h>
#include "arduinoFFT.h"

Servo arm;
Servo hand;
arduinoFFT FFT;

//////////////
// Have to redefine those below pins as we plugged
#define AIN1 49
#define BIN1 51
#define AIN2 47
#define BIN2 53
#define PWMA 3
#define PWMB 4
//motor1=right
////////

//ultrasonic sensor read
//////ultrasonic pins
#define lftrig 10
#define lfecho 11
#define rftrig 8
#define rfecho 9

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Assign pin numbers to each servo
const int armPin = 12;
const int handPin = 13;

// Constants
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A15, A14, A13, A12, A11, A10, A9, A8}; // Analog pins for the remaining sensors
int IR_weight[8] = { -50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];

// PID control parameters
float Kp = 4 ; // Proportional term
float Ki = 0; // Integral term
float Kd = 15; // Derivative term
float sum;
int increment_count = 0;
int wall_count = 0;
long ldistance;
long rdistance;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 95;

//////Mic
const int microphonePin = A4;  // Adjust the pin according to your setup

const uint16_t samples = 64;  // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 5000;
const int detectionDuration = 5000;  // Adjust as needed (in milliseconds)

const double detectionMinFrequency = 560;  // Adjust based on your environment
const double detectionMaxFrequency = 600;  // Adjust based on your environment
double averageFrequency;
double currentFrequency;

#define stopLED 39
#define goLED 37

unsigned long lastDetectionTime = 0;

double vReal[samples];
double vImag[samples];

const int maxStoredValues = 30;  // Adjust based on your needs
double storedFrequencies[maxStoredValues];
double storedFrequenciesSub[5];
int storedValuesIndex = 0;
//////////////////////////

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();
//Maze
void turnLeft_maze(int *sensorValues);
void turnRight_maze(int *sensorValues);
void noLine_maze();
void maze(int *sensorValues);
void towards_box(int *sensorValues);

void setup() {
  // Initialize analog sensor pins as input
  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(lftrig, OUTPUT);
  pinMode(lfecho, INPUT);
  pinMode(rftrig, OUTPUT);
  pinMode(rfecho, INPUT);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
  digitalWrite(23, LOW);
  digitalWrite(25, LOW);
  //Servo
  arm.attach(armPin);
  hand.attach(handPin);
  zip_mode();

  pinMode(stopLED, OUTPUT);
  pinMode(goLED, OUTPUT);

  digitalWrite(goLED, LOW);
  digitalWrite(stopLED, LOW);

  Serial.begin(9600);
}

void loop() {
  readSensors(sensorValues);
  lUltrasonic_read();
  rUltrasonic_read();
  if (increment_count == 0) {
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
      increment(sensorValues);
    }
  }
  else if (increment_count == 1) {
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
      motor1.stop();
      motor2.stop();
      motor1.setSpeed(85);   // Set right motor speed
      motor1.backward();       // Move right motor forward
      motor2.setSpeed(85);   // Set left motor speed
      motor2.forward();
      delay(80);
      motor1.stop();
      motor2.stop();
      increment(sensorValues);
    }
    else {
      line_follow(sensorValues);
    }
  }
  else if (increment_count == 2) {
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
      increment(sensorValues);
    }
    else {
      wall_follow(sensorValues);
    }
  }
  else if (increment_count == 3) {
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
      increment(sensorValues);
    }
    else {
      line_follow(sensorValues);
    }
  }
  else if (increment_count == 4) {
    motor1.stop();
    motor2.stop();
    delay(15000);
  }
  else {
    motor1.stop();
    motor2.stop();
    delay(15000);
  }
}

void increment(int *sensorValues) {
  motor1.stop();
  motor2.stop();
  delay(2000);
  white(sensorValues);

}

void increment_back(int *sensorValues) {
  motor1.stop();
  motor2.stop();
  delay(500);
  white_back(sensorValues);

}

void line_follow(int *sensorValues) {
  if (isImmediateTurn(sensorValues)) {
    motor1.stop();
    motor2.stop();
    //digitalWrite(26, HIGH);
    //delay(2000);
    turn(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    digitalWrite(30, HIGH);
    noLine(sensorValues);
    //digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, HIGH);
    digitalWrite(30, LOW);
  }
}

void line_follow_back(int *sensorValues) {
  if (isImmediateTurn(sensorValues)) {
    motor1.stop();
    motor2.stop();
    digitalWrite(26, HIGH);
    //delay(2000);
    turn(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    digitalWrite(30, HIGH);
    noLine_back(sensorValues);
    //digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow_back(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, HIGH);
    digitalWrite(30, LOW);
  }
}

void alarm_line_follow(int *sensorValues) {
  lfspeed = 80;
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    motor1.stop();
    motor2.stop();
    motor1.setSpeed(85);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(85);   // Set left motor speed
    motor2.forward();
    delay(80);
    motor1.stop();
    motor2.stop();
    turnLeft_T(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    motor1.stop();
    motor2.stop();
    motor1.setSpeed(85);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(85);   // Set left motor speed
    motor2.forward();
    delay(80);
    turnRight_T(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    digitalWrite(30, HIGH);
    noLine(sensorValues);
    //digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, HIGH);
    digitalWrite(30, LOW);
  }
}

void wall_follow(int *sensorValues) {
  lUltrasonic_read();
  rUltrasonic_read();
  Kp = 3.75 ; // Proportional term for wall following
  Ki = 0; // Integral term for wall following
  Kd = 15; // Derivative term for wall following
  lfspeed = 77;
  if (wall_count == 2) {
    increment_count += 1;
    Kp = 4 ; // Proportional term
    Ki = 0; // Integral term
    Kd = 15; // Derivative term
    lfspeed = 95;
  }
  if (ldistance < 40 || rdistance < 40) {
    motor1.stop();
    motor2.stop();
    //digitalWrite(26, HIGH);
    delay(1500);
    if (wall_count == 0) {
      wallLeft(sensorValues);
      //delay(2000);
      wallforward(sensorValues);
      motor1.setSpeed(85);   // Set right motor speed
      motor1.forward();       // Move right motor forward
      motor2.setSpeed(85);   // Set left motor speed
      motor2.forward();
      delay(250);
      turnLeft(sensorValues);
      wall_count += 1;
    }
    else if (wall_count == 1) {
      wallRight(sensorValues);
      //delay(2000);
      wallforward(sensorValues);
      motor1.setSpeed(85);   // Set right motor speed
      motor1.forward();       // Move right motor forward
      motor2.setSpeed(85);   // Set left motor speed
      motor2.forward();
      delay(250);
      turnRight(sensorValues);
      wall_count += 1;
    }
  }
  else if (isImmediateTurn(sensorValues)) {
    motor1.stop();
    motor2.stop();
    //delay(2000);
    turn(sensorValues);
    digitalWrite(8, LOW);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    noLine(sensorValues);
    digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
  }
}

void maze(int *sensorValues) {
  lfspeed = 55;
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    motor1.stop();
    motor2.stop();
    motor1.setSpeed(85);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(85);   // Set left motor speed
    motor2.forward();
    delay(80);
    motor1.stop();
    motor2.stop();
    readSensors(sensorValues);
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
      motor1.stop();
      motor2.stop();
      delay(80);
      increment_count += 1;
    }
    else {
      turnLeft_T(sensorValues);
      turnLeft(sensorValues);
      motor1.stop();
      motor2.stop();
      delay(80);
      motor1.setSpeed(85);   // Set right motor speed
      motor1.backward();       // Move right motor forward
      motor2.setSpeed(85);   // Set left motor speed
      motor2.backward();
      delay(250);
    }
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    motor1.stop();
    motor2.stop();
    motor1.setSpeed(85);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(85);   // Set left motor speed
    motor2.forward();
    delay(120);
    motor1.stop();
    motor2.stop();
    readSensors(sensorValues);
    if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
      turnRight_T(sensorValues);
      turnRight(sensorValues);
      motor1.stop();
      motor2.stop();
      delay(80);
      motor1.setSpeed(85);   // Set right motor speed
      motor1.backward();       // Move right motor forward
      motor2.setSpeed(85);   // Set left motor speed
      motor2.backward();
      delay(250);
      maze(sensorValues);
    }
    else {
      maze(sensorValues);
    }
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    digitalWrite(30, HIGH);
    noLine_maze(sensorValues);
    //digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
  }
}

void alarm_path(int *sensorValues) {
  Serial.println(currentFrequency);
  Serial.println(averageFrequency);
  frequencyValue(storedFrequencies);
  calculateArrayAverage(storedFrequencies, maxStoredValues);
  if ((currentFrequency >= detectionMinFrequency && currentFrequency <= detectionMaxFrequency)) {
    motor1.stop();
    motor2.stop();
  }
  else if (!(averageFrequency >= detectionMinFrequency && averageFrequency <= detectionMaxFrequency)) {
    alarm_line_follow(sensorValues);
  }
}

void task_4_begin(int *sensorValues) {
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    while (whitedetect) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
      if (!whitedetect) {
        break;
      }
      motor1.setSpeed(90);   // Set right motor speed
      motor1.forward();       // Move right motor forward
      motor2.setSpeed(90);   // Set left motor speed
      motor2.forward();      // Move left motor backward for a sharper turn
    }
    motor1.stop();
    motor2.stop();
    delay(200);
    increment_count += 1;
  }
  else {
    line_follow(sensorValues);
  }
}

void towards_box(int *sensorValues) {
  lfspeed = 70;
  if (ldistance < 14 || rdistance < 14) {
    motor1.stop();
    motor2.stop();
    digitalWrite(26, HIGH);
    delay(2000);
    digitalWrite(26, LOW);
    increment_count += 1;
  }
  else if (isImmediateTurn(sensorValues)) {
    motor1.stop();
    motor2.stop();
    //delay(2000);
    turn(sensorValues);
    digitalWrite(8, LOW);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    noLine(sensorValues);
    digitalWrite(8, LOW);
  }
  else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
  }
}

void box_picking() {
  delay(2000);
  servo_down();
  delay(2000);
  motor1.setSpeed(80);   // Set right motor speed
  motor1.forward();       // Move right motor forward
  motor2.setSpeed(80);   // Set left motor speed
  motor2.forward();
  delay(500);
  motor1.stop();
  motor2.stop();
  delay(500);
  pick_the_box();
  increment_count += 1;
}

void task_4_position(int *sensorValues) {
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    while (whitedetect) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
      if (!whitedetect) {
        break;
      }
      motor1.setSpeed(70);   // Set right motor speed
      motor1.backward();       // Move right motor forward
      motor2.setSpeed(70);   // Set left motor speed
      motor2.backward();      // Move left motor backward for a sharper turn
    }
    motor1.stop();
    motor2.stop();
    delay(200);
    increment_count += 1;
  }
  else {
    increment_count += 1;
  }
}
void task_4_back(int *sensorValues) {
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    motor1.stop();
    motor2.stop();
    delay(2000);
    increment_count += 1;
  }
  else {
    motor1.setSpeed(80);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(80);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
  }
}

void pathselection(int *sensorValues) {
  lfspeed = 80;
  delay(200);
  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    increment_count += 1;
    delay(200);
  }
  else {
    line_follow(sensorValues);
  }
}

void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line
  }
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  // Loop through all sensors
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) { // Assuming 1 indicates line detected
      position += IR_weight[(i)];
      onLine++;
    }
  }

  // If no line is detected by any sensor, use the last known error value
  if (onLine == 0) {
    // If previous error is not available, assume the line is straight ahead
    error = -previousError;
  }
  else {
    // Calculate the average position of the line
    position /= onLine;
    // Calculate error based on sensor position
    error = 0 - position;
  }
  for (int i = 49; i > 0; i--) {
    errorArray[i] = errorArray[i - 1];
  }
  errorArray[0] = error;
  // PID terms
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  // Calculate PID value
  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);


  return pidValue;
}

void PID_Linefollow(float pidValue) {
  lsp = lfspeed - pidValue;
  rsp = lfspeed + pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = 255;
  }
  motor_drive(lsp, rsp);
}

void motor_drive(float left, float right) {
  int absRight = abs(right); // Absolute value for right speed
  int absLeft = abs(left);   // Absolute value for left speed

  if (left > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }

  if (right > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }
}

void PID_Linefollow_back(float pidValue) {
  lsp = lfspeed - pidValue;
  rsp = lfspeed + pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = 255;
  }
  motor_drive_back(lsp, rsp);
}

void motor_drive_back(float left, float right) {
  int absRight = abs(right); // Absolute value for right speed
  int absLeft = abs(left);   // Absolute value for left speed

  if (left < 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }

  if (right < 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }
}

bool isImmediateTurn(int *sensorValues) {
  bool turnLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1;
  bool turnRight = sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return (turnLeft || turnRight);
}
void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    turnLeft(sensorValues);

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnRight(sensorValues);
  }
  else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(100);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(100);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
    //digitalWrite(26, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(100);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(100);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnLeft_T(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(95);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(120);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight_T(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(120);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(120);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void noLine(int *sensorValues) {
  float firstNonZeroValue;
  for (int i = 0; i < 50; i++)
  {
    if (errorArray[i] != 0) {
      firstNonZeroValue = errorArray[i];
      break; //
    }
  }

  if (firstNonZeroValue < 0) {
    turnRight(sensorValues);

  }
  else if (firstNonZeroValue > 0) {
    turnLeft(sensorValues);

  }
}

void turnLeft_back(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.stop();       // Move right motor forward
    motor2.setSpeed(85);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
    digitalWrite(26, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight_back(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(85);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.stop();;      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void noLine_back(int *sensorValues) {
  float firstNonZeroValue;
  for (int i = 0; i < 50; i++)
  {
    if (errorArray[i] != 0) {
      firstNonZeroValue = errorArray[i];
      break; //
    }
  }

  if (firstNonZeroValue < 0) {
    turnRight_back(sensorValues);

  }
  else if (firstNonZeroValue > 0) {
    turnLeft_back(sensorValues);

  }
}

void white(int *sensorValues) {
  //  motor1.setSpeed(55);   // Set right motor speed
  //  motor1.forward();       // Move right motor forward
  //  motor2.setSpeed(55);   // Set left motor speed
  //  motor2.forward();
  //  delay(400);
  //  motor1.stop();
  //  motor2.stop();
  //  delay(200);
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    motor1.setSpeed(80);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(80);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  increment_count += 1;
}

void white_back(int *sensorValues) {
  //  motor1.setSpeed(55);   // Set right motor speed
  //  motor1.forward();       // Move right motor forward
  //  motor2.setSpeed(55);   // Set left motor speed
  //  motor2.forward();
  //  delay(400);
  //  motor1.stop();
  //  motor2.stop();
  //  delay(200);
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    motor1.setSpeed(90);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(90);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  increment_count += 1;
}

long lUltrasonic_read() {
  digitalWrite(lftrig, LOW);
  delayMicroseconds(2);
  digitalWrite(lftrig, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (lfecho, HIGH);
  ldistance = time / 29 / 2;
  return ldistance;
}
long rUltrasonic_read() {
  digitalWrite(rftrig, LOW);
  delayMicroseconds(2);
  digitalWrite(rftrig, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (rfecho, HIGH);
  rdistance = time / 29 / 2;
  return rdistance;
}

void wallLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool leftturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!leftturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    leftturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (leftturn || whitedetect) {
      break;
    }
    motor1.setSpeed(255);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(255);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void wallRight(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool rightturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!rightturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    rightturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (rightturn || whitedetect) {
      break;
    }
    motor2.setSpeed(255);   // Set right motor speed
    motor2.forward();       // Move right motor forward
    motor1.setSpeed(255);   // Set left motor speed
    motor1.backward();      // Move left motor backward for a sharper turn
    //digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void wallforward(int *sensorValues) {
  sensorValues[SensorCount];
  readSensors(sensorValues);
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[3] == 1 && sensorValues[4] == 1;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn =  sensorValues[3] == 1 && sensorValues[4] == 1;
    //whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn) {
      break;
    }
    motor1.setSpeed(55);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(55);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(30, HIGH);

  }
  digitalWrite(30, LOW);
  motor1.stop();
  motor2.stop();
  delay(2000);
}

void moveServo(Servo servo, int targetPosition) {
  int currentPosition = servo.read();
  if (currentPosition < targetPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos += 1) {
      servo.write(pos);
      delay(20);  // adjust delay for desired speed
    }
  } else {
    for (int pos = currentPosition; pos >= targetPosition; pos -= 1) {
      servo.write(pos);
      delay(20);  // adjust delay for desired speed
    }
  }
}

void zip_mode() {
  moveServo(arm, 90);
  moveServo(hand, 90);
}

void servo_down() {
  moveServo(hand, 0);
  moveServo(arm, 175);
}
void pick_the_box() {
  moveServo(hand, 170);
  delay(2000);
  moveServo(arm, 85);
  delay(500);
}

void place_the_box() {
  moveServo(arm, 180);
  delay(500);
  moveServo(hand, 0);
  delay(500);
}

double frequencyValue(double *storedFrequencies) {
  // Read input from the microphone
  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = analogRead(microphonePin);
    vImag[i] = 0.0;
  }

  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  currentFrequency = FFT.MajorPeak();
  //Serial.println("Main Frequency: " + String(currentFrequency, 6) + " Hz");

  // Shift the existing values in the array
  for (int i = 0; i < maxStoredValues - 1; i++) {
    storedFrequencies[i] = storedFrequencies[i + 1];
  }
  // Put the latest frequency at the end of the array
  storedFrequencies[maxStoredValues - 1] = currentFrequency;
}

double calculateArrayAverage(const double array[], int length) {
  if (length == 0) {
    return 0.0;  // Avoid division by zero
  }

  double sum = 0.0;
  for (int i = 0; i < length; i++) {
    sum += array[i];
  }

  averageFrequency = sum / length;
}
