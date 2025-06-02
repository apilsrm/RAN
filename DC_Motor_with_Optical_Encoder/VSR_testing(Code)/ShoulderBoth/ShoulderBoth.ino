#include <EEPROM.h>

// Motor structure to hold configuration for each motor
struct Motor {
  int sensorPin;
  int upperLimitPin;
  int lowerLimitPin;
  int pwmPin;
  int in1Pin;
  int in2Pin;
  int enablePin;
  volatile int pos;
  int maxPos;
  int minPos;
  int initialPos; // Removed 'const' to allow brace initialization
  int address; // EEPROM address for storing position
};

// Pin definitions for right motor
#define RIGHT_SENSOR_PIN 20
#define RIGHT_UPPER_LIMIT_PIN 8
#define RIGHT_LOWER_LIMIT_PIN 9
#define RIGHT_PWM 11
#define RIGHT_IN1 25
#define RIGHT_IN2 23
#define RIGHT_EN A1 

// Pin definitions for left motor
#define LEFT_SENSOR_PIN 18
#define LEFT_UPPER_LIMIT_PIN 7 
#define LEFT_LOWER_LIMIT_PIN 6 
#define LEFT_PWM 10
#define LEFT_IN1 27
#define LEFT_IN2 29
#define LEFT_EN A0 // 

// Motor parameters (same for both motors)
const int MAX_POS = 30;
const int MIN_POS = 0;
const int UPWARD_MAX_SPEED = 100;
const int UPWARD_MEDIUM_SPEED = 90;
const int DOWNWARD_LOW_SPEED = 50;
const int DOWNWARD_MAX_SPEED = 80;
const int MIN_SPEED = 60;
const int ACCEL_STEPS = 5;
const int DOWNWARD_DELAY = 10;
const float Kp = 0.5;
const float Ki = 0.01;
const float Kd = 0.1;

// Motor instances
Motor rightMotor = {RIGHT_SENSOR_PIN, RIGHT_UPPER_LIMIT_PIN, RIGHT_LOWER_LIMIT_PIN, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, RIGHT_EN, 0, MAX_POS, MIN_POS, 20, 0};
Motor leftMotor = {LEFT_SENSOR_PIN, LEFT_UPPER_LIMIT_PIN, LEFT_LOWER_LIMIT_PIN, LEFT_PWM, LEFT_IN1, LEFT_IN2, LEFT_EN, 0, MAX_POS, MIN_POS, 20, 1};

// Function prototypes
void moveToPosition(Motor &motor, int targetPos);
void moveUpward(Motor &motor, int targetPos);
void moveDownward(Motor &motor, int targetPos);
void applyBraking(Motor &motor, int delayTime);
void setMotor(Motor &motor, int dir, int pwmVal);
void readSensor(Motor &motor);
void handleLimitHit(Motor &motor);
void calibratePosition(Motor &motor);

void setup() {
  Serial.begin(9600);

  // Initialize right motor pins
  pinMode(rightMotor.sensorPin, INPUT);
  pinMode(rightMotor.upperLimitPin, INPUT_PULLUP);
  pinMode(rightMotor.lowerLimitPin, INPUT_PULLUP);
  pinMode(rightMotor.pwmPin, OUTPUT);
  pinMode(rightMotor.in1Pin, OUTPUT);
  pinMode(rightMotor.in2Pin, OUTPUT);
  pinMode(rightMotor.enablePin, OUTPUT);
  digitalWrite(rightMotor.enablePin, HIGH);
  digitalWrite(rightMotor.in1Pin, LOW);
  digitalWrite(rightMotor.in2Pin, LOW);
  analogWrite(rightMotor.pwmPin, 0);

  // Initialize left motor pins
  pinMode(leftMotor.sensorPin, INPUT);
  pinMode(leftMotor.upperLimitPin, INPUT_PULLUP);
  pinMode(leftMotor.lowerLimitPin, INPUT_PULLUP);
  pinMode(leftMotor.pwmPin, OUTPUT);
  pinMode(leftMotor.in1Pin, OUTPUT);
  pinMode(leftMotor.in2Pin, OUTPUT);
  pinMode(leftMotor.enablePin, OUTPUT);
  digitalWrite(leftMotor.enablePin, HIGH);
  digitalWrite(leftMotor.in1Pin, LOW);
  digitalWrite(leftMotor.in2Pin, LOW);
  analogWrite(leftMotor.pwmPin, 0);

  // Attach interrupts for both motors
  attachInterrupt(digitalPinToInterrupt(rightMotor.sensorPin), [] { readSensor(rightMotor); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftMotor.sensorPin), [] { readSensor(leftMotor); }, CHANGE);

  // Calibrate both motors
  calibratePosition(rightMotor);
  calibratePosition(leftMotor);

  // Load positions from EEPROM
  rightMotor.pos = EEPROM.read(rightMotor.address);
  leftMotor.pos = EEPROM.read(leftMotor.address);
  Serial.print("Right Motor - Loaded position: ");
  Serial.println(rightMotor.pos);
  Serial.print("Left Motor - Loaded position: ");
  Serial.println(leftMotor.pos);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int rightTarget = 0, leftTarget = 0;

    // Expect input format: "R<pos>,L<pos>" (e.g., "R10,L15")
    if (input.startsWith("R") && input.indexOf(",L") != -1) {
      rightTarget = input.substring(1, input.indexOf(",L")).toInt();
      leftTarget = input.substring(input.indexOf(",L") + 2).toInt();

      // Validate targets
      bool validRight = rightTarget >= rightMotor.minPos && rightTarget <= rightMotor.maxPos;
      bool validLeft = leftTarget >= leftMotor.minPos && leftTarget <= leftMotor.maxPos;

      if (validRight && validLeft) {
        // Move both motors concurrently
        bool rightMoving = rightMotor.pos != rightTarget;
        bool leftMoving = leftMotor.pos != leftTarget;

        while (rightMoving || leftMoving) {
          if (rightMoving) {
            moveToPosition(rightMotor, rightTarget);
            if (rightMotor.pos == rightTarget) {
              rightMoving = false;
              setMotor(rightMotor, 0, 0);
              Serial.print("Right Motor - Moved to position: ");
              Serial.println(rightMotor.pos);
              EEPROM.write(rightMotor.address, rightMotor.pos);
            }
          }
          if (leftMoving) {
            moveToPosition(leftMotor, leftTarget);
            if (leftMotor.pos == leftTarget) {
              leftMoving = false;
              setMotor(leftMotor, 0, 0);
              Serial.print("Left Motor - Moved to position: ");
              Serial.println(leftMotor.pos);
              EEPROM.write(leftMotor.address, leftMotor.pos);
            }
          }
          // Check limit switches
          if (digitalRead(rightMotor.upperLimitPin) == LOW || digitalRead(rightMotor.lowerLimitPin) == LOW) {
            handleLimitHit(rightMotor);
            rightMoving = false;
          }
          if (digitalRead(leftMotor.upperLimitPin) == LOW || digitalRead(leftMotor.lowerLimitPin) == LOW) {
            handleLimitHit(leftMotor);
            leftMoving = false;
          }
          delay(10); // Small delay to prevent excessive CPU usage
        }
      } else {
        Serial.println("Invalid position(s)! Must be between 0 and 30.");
      }
    } else {
      Serial.println("Invalid input format! Use 'R<pos>,L<pos>'");
    }
    while (Serial.available()) Serial.read();
  }

  // Check limit switches continuously
  if (digitalRead(rightMotor.upperLimitPin) == LOW || digitalRead(rightMotor.lowerLimitPin) == LOW) {
    handleLimitHit(rightMotor);
  }
  if (digitalRead(leftMotor.upperLimitPin) == LOW || digitalRead(leftMotor.lowerLimitPin) == LOW) {
    handleLimitHit(leftMotor);
  }
}

void moveToPosition(Motor &motor, int targetPos) {
  Serial.print("Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.print(" moving to position: ");
  Serial.println(targetPos);
  if (motor.pos < targetPos) {
    moveUpward(motor, targetPos);
  } else if (motor.pos > targetPos) {
    moveDownward(motor, targetPos);
  }
  setMotor(motor, 0, 0);
  Serial.print("Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.print(" target reached, pos: ");
  Serial.println(motor.pos);
}

void moveUpward(Motor &motor, int targetPos) {
  float integral = 0;
  float previousError = 0;

  while (motor.pos < targetPos) {
    if (digitalRead(motor.upperLimitPin) == LOW) {
      handleLimitHit(motor);
      break;
    }

    float error = targetPos - motor.pos;
    integral += error;
    float derivative = error - previousError;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    int pwmVal;
    if (motor.pos < 5) {
      pwmVal = map(motor.pos, 0, 5, MIN_SPEED, UPWARD_MEDIUM_SPEED);
    } else if (motor.pos < 20) {
      pwmVal = UPWARD_MAX_SPEED;
    } else {
      pwmVal = UPWARD_MEDIUM_SPEED;
    }
    pwmVal = constrain(pwmVal + pidOutput, MIN_SPEED, UPWARD_MAX_SPEED);

    setMotor(motor, 1, pwmVal);
    Serial.print("Motor at pin ");
    Serial.print(motor.pwmPin);
    Serial.print(" Upward, Pos: ");
    Serial.print(motor.pos);
    Serial.print(", PWM: ");
    Serial.println(pwmVal);
    previousError = error;
    delay(10);
  }
  EEPROM.write(motor.address, motor.pos);
}

void moveDownward(Motor &motor, int targetPos) {
  float integral = 0;
  float previousError = 0;

  while (motor.pos > targetPos) {
    if (digitalRead(motor.lowerLimitPin) == LOW) {
      handleLimitHit(motor);
      break;
    }

    float error = targetPos - motor.pos;
    integral += error;
    float derivative = error - previousError;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    int pwmVal;
    if (motor.pos > 25) {
      pwmVal = map(motor.pos, 30, 25, 50, DOWNWARD_LOW_SPEED);
    } else if (motor.pos > 10) {
      pwmVal = DOWNWARD_LOW_SPEED;
    } else {
      pwmVal = DOWNWARD_MAX_SPEED;
    }
    int stepsToTarget = motor.pos - targetPos;
    if (stepsToTarget <= 5) {
      applyBraking(motor, 10);
    }

    pwmVal = constrain(pwmVal + pidOutput, MIN_SPEED, DOWNWARD_MAX_SPEED);
    setMotor(motor, -1, pwmVal);
    Serial.print("Motor at pin ");
    Serial.print(motor.pwmPin);
    Serial.print(" Downward, Pos: ");
    Serial.print(motor.pos);
    Serial.print(", PWM: ");
    Serial.println(pwmVal);
    previousError = error;
    delay(DOWNWARD_DELAY);
  }
}

void applyBraking(Motor &motor, int delayTime) {
  digitalWrite(motor.in1Pin, HIGH);
  digitalWrite(motor.in2Pin, HIGH);
  analogWrite(motor.pwmPin, 0);
  delay(delayTime);
}

void setMotor(Motor &motor, int dir, int pwmVal) {
  analogWrite(motor.pwmPin, pwmVal);
  if (dir == -1) {
    digitalWrite(motor.in1Pin, LOW);
    digitalWrite(motor.in2Pin, HIGH);
  } else if (dir == 1) {
    digitalWrite(motor.in1Pin, HIGH);
    digitalWrite(motor.in2Pin, LOW);
  } else {
    digitalWrite(motor.in1Pin, LOW);
    digitalWrite(motor.in2Pin, LOW);
    analogWrite(motor.pwmPin, 0);
  }
}

void readSensor(Motor &motor) {
  static int lastState[2] = {HIGH, HIGH}; // Store last state for each motor
  static unsigned long lastInterruptTime[2] = {0, 0};
  const unsigned long debounceDelay = 40;

  int motorIndex = (motor.sensorPin == rightMotor.sensorPin) ? 0 : 1;
  int currentState = digitalRead(motor.sensorPin);
  unsigned long currentTime = millis();

  if ((currentState != lastState[motorIndex]) && (currentTime - lastInterruptTime[motorIndex] >= debounceDelay)) {
    if (digitalRead(motor.in1Pin) == LOW && digitalRead(motor.in2Pin) == HIGH) {
      if (digitalRead(motor.lowerLimitPin) == HIGH) {
        motor.pos--;
      }
    } else if (digitalRead(motor.in1Pin) == HIGH && digitalRead(motor.in2Pin) == LOW) {
      if (digitalRead(motor.upperLimitPin) == HIGH) {
        motor.pos++;
      }
    }
    lastInterruptTime[motorIndex] = currentTime;
  }
  lastState[motorIndex] = currentState;
}

void handleLimitHit(Motor &motor) {
  setMotor(motor, 0, 0);
  Serial.print("Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.print(" Limit hit at position: ");
  Serial.println(motor.pos);
  if (digitalRead(motor.upperLimitPin) == LOW) {
    Serial.println("Upper limit hit, moving downward...");
    setMotor(motor, -1, DOWNWARD_LOW_SPEED);
    delay(500);
    setMotor(motor, 0, 0);
    calibratePosition(motor);
  } else if (digitalRead(motor.lowerLimitPin) == LOW) {
    Serial.println("Lower limit hit, moving upward...");
    setMotor(motor, 1, UPWARD_MEDIUM_SPEED);
    delay(500);
    setMotor(motor, 0, 0);
    calibratePosition(motor);
  }
}

void calibratePosition(Motor &motor) {
  Serial.print("Calibrating Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.println(": Finding lower limit...");
  setMotor(motor, -1, 70);
  while (digitalRead(motor.lowerLimitPin) == HIGH) {
    delay(DOWNWARD_DELAY);
  }
  setMotor(motor, 0, 0);

  motor.pos = 0;
  motor.minPos = 0;
  Serial.print("Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.print(" Lower limit hit, set as minPos: ");
  Serial.println(motor.minPos);

  delay(500);

  Serial.print("Motor at pin ");
  Serial.print(motor.pwmPin);
  Serial.println(" Moving to initial position...");
  moveToPosition(motor, motor.initialPos);
  motor.pos = motor.initialPos;
  EEPROM.write(motor.address, motor.pos);
}