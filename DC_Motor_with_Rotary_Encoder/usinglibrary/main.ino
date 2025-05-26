#include <Arduino.h>
#include "MotorControl.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Enter steps to move (positive/negative):");

  initializeMotor();
  initializeEncoder();

  Serial.println("System initialized");
}

void loop() {
  static long lastPrintedPos = -1;
  if (encoderPos != lastPrintedPos) {
    lastPrintedPos = encoderPos;
  }

  processSerialCommand();

  if (motorRunning) {
    moveToTarget(encoderPos, targetPos);
  }

  encoder.Process(millis());
}
