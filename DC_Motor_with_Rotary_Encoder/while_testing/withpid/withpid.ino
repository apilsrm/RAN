// === Pin Definitions ===
#define INA 10
#define INB 9
#define PWM 5
#define EN A0

#define pinCLK 2
#define pinDT 3

// === Encoder Variables ===
volatile long encoderPos = 0;
volatile int lastCLK = 0;

// === Motor Control Parameters ===
long targetPosition = 0;
const float Kp = 2.0;
const float Ki = 0.05;
const float Kd = 1.0;
const int maxSpeed = 255;
const int minSpeed = 100;

// === PID Variables ===
float integral = 0;
float previousError = 0;

// === Timing ===
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 50; // ms
unsigned long moveStartTime = 0;
const unsigned long moveTimeout = 5000; // ms

void setup() {
  Serial.begin(9600);
  Serial.println("Enter target position (in steps):");

  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinCLK), readEncoder, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    targetPosition = Serial.parseInt();
    Serial.readStringUntil('\n'); // Clear buffer
    Serial.print("Target position set to: ");
    Serial.println(targetPosition);
    moveStartTime = millis(); // Reset timer
    integral = 0;             // Reset PID terms
    previousError = 0;
  }

  unsigned long now = millis();
  if (now - lastControlTime >= controlInterval) {
    updateMotor();
    lastControlTime = now;
  }
}

void readEncoder() {
  int clkState = digitalRead(pinCLK);
  int dtState = digitalRead(pinDT);

  if (clkState != lastCLK) {
    if (dtState == clkState) {
      encoderPos--;
    } else {
      encoderPos++;
    }
  }
  lastCLK = clkState;
}

void moveMotor(int direction, int speed) {
  speed = constrain(speed, 0, maxSpeed);

  if (direction == 1) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else if (direction == -1) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH); // Motor brake
    speed = 0;
  }

  analogWrite(PWM, speed);
}

void updateMotor() {
  static bool motorRunning = false;
  long currentPos;

  noInterrupts();
  currentPos = encoderPos;
  interrupts();

  long error = targetPosition - currentPos;
  float deltaTime = controlInterval / 1000.0;

  // === PID calculation ===
  integral += error * deltaTime;
  float derivative = (error - previousError) / deltaTime;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  int direction = (output > 0) ? 1 : (output < 0) ? -1 : 0;
  int speed = abs(output);
  if (speed < minSpeed && error != 0) speed = minSpeed;
  if (speed > maxSpeed) speed = maxSpeed;

  unsigned long now = millis();

  if (motorRunning && (now - moveStartTime > moveTimeout)) {
    moveMotor(0, 0);
    Serial.println("⚠️ Timeout: Target not reached.");
    motorRunning = false;
    return;
  }

  if (abs(error) < 1) {
    moveMotor(0, 0);
    if (motorRunning) {
      Serial.println("✅ Target reached.");
      motorRunning = false;
    }
  } else {
    moveMotor(direction, speed);
    if (!motorRunning) {
      moveStartTime = now;
      motorRunning = true;
    }

    // Print debug info only when moving
    Serial.print("Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Current: ");
    Serial.print(currentPos);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}
