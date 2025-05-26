#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "KY040rotary.h"

// Pin setup
#define INA 9
#define INB 10
#define PWM 5
#define EN A0
#define CLK 2
#define DT 3

// Motor control functions
void initializeMotor();
void initializeEncoder();
void moveMotor(int direction, int speed);
void moveToTarget(long currentPos, long targetPos);
void rotateInterruptHandler();
void onEncoderRight();
void onEncoderLeft();
void processSerialCommand();

// Global access to encoder and state
extern KY040 encoder;
extern volatile long encoderPos;
extern long targetPos;
extern bool motorRunning;

#endif
