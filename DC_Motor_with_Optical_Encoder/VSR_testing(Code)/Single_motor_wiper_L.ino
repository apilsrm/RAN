#include <EEPROM.h>

#define SENSOR_PIN 18
#define UPPER_LIMIT_PIN 7
#define LOWER_LIMIT_PIN 6
#define PWM 10
#define IN1 27
#define IN2 29
#define EN A0

volatile int pos = 0;
int maxPos = 30;
int minPos = 0;
const int INITIAL_POS = 20;
const int UPWARD_MAX_SPEED = 100;  // High speed for 5-20 steps
const int UPWARD_MEDIUM_SPEED = 90; // Medium speed for 20-30 steps
const int DOWNWARD_LOW_SPEED = 50;  // Low speed for 25-10 steps
const int DOWNWARD_MAX_SPEED = 80; // High speed for 10-0 steps
const int MIN_SPEED = 60;           // Minimum speed for soft start
const int ACCEL_STEPS = 5;          // Steps for soft start (0-5 and 30-25)
const int DOWNWARD_DELAY = 10;      // Delay for downward motion
const float Kp = 0.5;               // PID proportional gain
const float Ki = 0.01;              // PID integral gain
const float Kd = 0.1;               // PID derivative gain


void moveToPosition(int targetPos); 
void moveUpward(int targetPos);
void moveDownward(int targetPos);
void applyBraking(int delayTime);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readSensor();
void handleLimitHit();
void calibratePosition();


void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 0);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), readSensor, CHANGE);
  calibratePosition();
  pos = EEPROM.read(0);
  maxPos = 30;
  minPos = 0;
  Serial.print("Loaded position: ");
  Serial.print("maxPos:");
  Serial.print(maxPos);
  Serial.print(", minPos:");
  Serial.println(pos);
}

void loop() {
  if (Serial.available() > 0) {
    int target = Serial.parseInt();
    if (target >= minPos && target <= maxPos) {
      moveToPosition(target);
      Serial.print("Moved to position: ");
      Serial.println(pos);
      EEPROM.write(0, pos);
    } else {
      Serial.print("Invalid position! Must be between ");
      Serial.print(minPos);
      Serial.print(" and ");
      Serial.println(maxPos);
    }
    while (Serial.available()) Serial.read();
  }
  if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
    handleLimitHit();
  }
}

void moveToPosition(int targetPos) {
  Serial.print("Moving to position: ");
  Serial.println(targetPos);
  if (pos < targetPos) {
    moveUpward(targetPos);
  } else if (pos > targetPos) {
    
    moveDownward(targetPos);
  }
  setMotor(0, 0, PWM, IN1, IN2);
  Serial.print("Target reached, pos: ");
  Serial.println(pos);
}

void moveUpward(int targetPos) {
  float integral = 0;
  float previousError = 0;
  int stepsMoved = 0;

  while (pos < targetPos) {
    if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
    }

 
    float error = targetPos - pos;
    integral += error;
    float derivative = error - previousError;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    int pwmVal;

    // Trapezoidal velocity profile for upward motion
    if (pos < 5) { // 0-5 steps: Soft start
      pwmVal = map(pos, 0, 5, MIN_SPEED, UPWARD_MEDIUM_SPEED);
    } else if (pos < 20) { // 5-20 steps: High speed
      pwmVal = UPWARD_MAX_SPEED;
    } else { // 20-30 steps: Medium speed
      pwmVal = UPWARD_MEDIUM_SPEED;
    }
    // // Decelerate near target
    // int stepsToTarget = targetPos - pos;
    // if (stepsToTarget <= ACCEL_STEPS) {
    //   pwmVal = map(stepsToTarget, ACCEL_STEPS, 0, pwmVal, MIN_SPEED);
    // }
    pwmVal = constrain(pwmVal + pidOutput, MIN_SPEED, UPWARD_MAX_SPEED);

    setMotor(1, pwmVal, PWM, IN1, IN2);
    Serial.print("Upward, Pos: ");
    Serial.print(pos);
    Serial.print(", PWM: ");
    Serial.println(pwmVal);
    stepsMoved++;
    previousError = error;
    delay(10);
  }
  EEPROM.write(0, pos);

}

void moveDownward(int targetPos) {
  float integral = 0;
  float previousError = 0;
  int stepsMoved = 0;

  while (pos > targetPos) {
    if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
      
    }

    float error = targetPos - pos;
    integral += error;
    float derivative = error - previousError;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    int pwmVal;
    // Trapezoidal velocity profile for downward motion
    if (pos > 25) { // 30-25 steps: 
      pwmVal = map(pos, 30, 25, 50, DOWNWARD_LOW_SPEED);
      // pwmVal = 40;
      // applyBraking(1);
    } else if (pos > 10) { // 25-10 steps: Low speed
      // applyBraking(1);
      pwmVal = DOWNWARD_LOW_SPEED;
    } else { // 10-0 steps: High speed
      
      pwmVal = DOWNWARD_MAX_SPEED;
    }
    // // Decelerate near target
    // int stepsToTarget = pos - targetPos;
    // if (stepsToTarget <= ACCEL_STEPS) {
    //   pwmVal = map(stepsToTarget, ACCEL_STEPS, 0, pwmVal, MIN_SPEED);
    // }
    // pwmVal = constrain(pwmVal + pidOutput, MIN_SPEED, DOWNWARD_MAX_SPEED);

    // Apply braking near target
    // if (stepsToTarget <= 2) {
    //   applyBraking(50);
    // }

    setMotor(-1, pwmVal, PWM, IN1, IN2);
    // applyBraking(10);
    Serial.print("Downward, Pos: ");
    Serial.print(pos);
    Serial.print(", PWM: ");
    Serial.println(pwmVal);
    stepsMoved++;
    previousError = error;
    delay(DOWNWARD_DELAY);
  }
}

void applyBraking(int delayTime) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, 0);
  delay(delayTime);  // Use the passed delay time
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

void readSensor() {
  static int lastState = HIGH;
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 40;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {
      if (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
        pos= pos - 1;
        // Serial.print("Decrease: ");
        // Serial.println(pos);
      }
    } else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {
      if (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
       pos= pos + 1;
      //  Serial.print("Increase: ");
      //  Serial.println(pos);
      }
    }
    lastInterruptTime = currentTime;
  }
  lastState = currentState;
}

void handleLimitHit() {
  setMotor(0, 0, PWM, IN1, IN2);
  Serial.print("Limit hit at position: ");
  Serial.println(pos);
  if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
    Serial.println("Upper limit hit, moving downward...");
    setMotor(-1, DOWNWARD_LOW_SPEED, PWM, IN1, IN2);
    delay(500);
    setMotor(0, 0, PWM, IN1, IN2);
    calibratePosition();

  } else if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.println("Lower limit hit, moving upward...");
    setMotor(1, UPWARD_MEDIUM_SPEED, PWM, IN1, IN2);
    delay(500);
    setMotor(0, 0, PWM, IN1, IN2);
    calibratePosition();

  }
}

void calibratePosition() {
  Serial.println("Calibrating: Finding lower limit...");
  setMotor(-1, 60, PWM, IN1, IN2);
  while (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
    delay(DOWNWARD_DELAY);
  }
  setMotor(0, 0, PWM, IN1, IN2);

  pos = 0;
  minPos = 0;
  Serial.print("Lower limit hit, set as minPos: ");
  Serial.println(minPos);
  
  delay(500);


  Serial.println("Moving to initial position...");
  moveToPosition(INITIAL_POS);
  pos = INITIAL_POS;
  EEPROM.write(0, pos);
}
