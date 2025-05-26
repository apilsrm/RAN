#include "MotorControl.h"
#include <Arduino.h>
#include "KY040rotary.h"

// Encoder object
KY040 encoder(CLK, DT, -1);

// Variables
volatile long encoderPos = 0;
long targetPos = 0;
bool motorRunning = false;

// Parameters
const int fixedSpeed = 50;
const int minRampSpeed = 20;
const int rampZone = 20;
const int targetTolerance = 0;
const long maxPos = 500;
const long minPos = -500;

void onEncoderRight() {
  if (encoderPos < maxPos) {
    encoderPos++;
    Serial.print("Position +: ");
    Serial.println(encoderPos);
  }
}

void onEncoderLeft() {
  if (encoderPos > minPos) {
    encoderPos--;
    Serial.print("Position - : ");
    Serial.println(encoderPos);
  }
}

void rotateInterruptHandler() {
  encoder.HandleRotateInterrupt();
}

void moveMotor(int direction, int speed) {
  analogWrite(PWM, speed);
  if (direction == -1) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (direction == 1) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
}

void moveToTarget(long currentPos, long targetPos) {
  if (currentPos > maxPos || currentPos < minPos) {
    moveMotor(0, 0);
    motorRunning = false;
    Serial.println("Error: Position limit reached!");
    Serial.print("Current position: ");
    Serial.println(currentPos);
    return;
  }

  long error = targetPos - currentPos;

  if (abs(error) <= targetTolerance) {
    moveMotor(0, 0);
    motorRunning = false;
    Serial.println("Target reached!");
    Serial.print("Current position: ");
    Serial.println(currentPos);
    return;
  }

  int direction = (error > 0) ? 1 : -1;

  int speed = fixedSpeed;
  if (abs(error) <= rampZone) {
    speed = minRampSpeed + (fixedSpeed - minRampSpeed) * abs(error) / rampZone;
    // Serial.print("Speed: ");
    // Serial.println(speed);
  }

  moveMotor(direction, speed);
}

void initializeMotor() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
}

void initializeEncoder() {
  if (!encoder.Begin(NULL, rotateInterruptHandler)) {
    Serial.println("Failed to initialize KY-040 encoder");
    while (1);
  }
  encoder.OnButtonRight(onEncoderRight);
  encoder.OnButtonLeft(onEncoderLeft);
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int steps = input.toInt();
    if (input.length() > 0 && steps == 0 && input != "0") {
      Serial.println("Invalid input. Enter a number.");
    } else {
      targetPos = encoderPos + steps;
      motorRunning = true;
      Serial.print("Moving to target: ");
      Serial.println(targetPos);
    }
  }
}
