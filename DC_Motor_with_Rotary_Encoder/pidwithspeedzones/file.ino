//  (VNH2SP30 Monster motor driver and  HW-040 rotary encoder )
// === Pin Definitions ===
#define INA 9
#define INB 10
#define PWM 5
#define EN A0
#define pinCLK 2
#define pinDT 3

// === Encoder Variables ===
volatile long encoderPos = 0;
volatile int lastCLK = 0;

// === PID Control Variables ===
long targetPosition = 0;
float Kp = 2.0;
float Ki = 0.2;
float Kd = 1.2;

int maxSpeed = 200;
int minSpeed = 50;

float integral = 0;
long lastError = 0;

// === Timing Variables ===
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 30; // ms

// Optional: Timed Speed Modulation (default:false)
bool useSpeedModulation = false;
unsigned long lastModTime = 0;
int speedPhase = 0;

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
    Serial.readStringUntil('\n');
    Serial.print("Target position set to: ");
    Serial.println(targetPosition);
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
    encoderPos += (dtState == clkState) ? -1 : 1;
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
    digitalWrite(INB, HIGH);
    speed = 0;
  }
  analogWrite(PWM, speed);
}

void updateMotor() {
  long currentPos;
  noInterrupts();
  currentPos = encoderPos;
  interrupts();

  long error = targetPosition - currentPos;
  int direction = (error > 0) ? 1 : (error < 0) ? -1 : 0;
  long absError = abs(error);

  // === PID Calculation ===
  integral += error * (controlInterval / 1000.0);  // Convert ms to sec
  float derivative = (error - lastError) / (controlInterval / 1000.0);
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int speed = abs(output);

  // === Apply Speed Zones ===
  if (absError > 8) {
    speed = constrain(speed, 100, 150);  // Fast
  } else if (absError > 5) {
    speed = constrain(speed, 80, 120);   // Cruise
  } else {
    speed = constrain(speed, minSpeed, 80); // Approach
  }

  // ===  Timed Speed Modulation ===
  if (useSpeedModulation) {
    if (millis() - lastModTime >= 30) {
      speedPhase = !speedPhase;
      lastModTime = millis();
    }
    if (speedPhase == 0) speed -= 20;
    else speed += 20;
    speed = constrain(speed, minSpeed, maxSpeed);
  }

  if (absError <= 0) {
    moveMotor(0, 0);
  } else {
    moveMotor(direction, speed);
  }

  // Debug only when moving
  if (absError > 0) {
    Serial.print("Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Pos: ");
    Serial.print(currentPos);
    Serial.print(" | Err: ");
    Serial.print(error);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}
