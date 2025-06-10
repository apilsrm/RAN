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

// === Motor Control Parameters ===
long targetPosition = 0;
const float Kp = 2.0; // Proportional gain
const int maxSpeed = 255;
const int minSpeed = 100;

unsigned long moveStartTime = 0;
const unsigned long moveTimeout = 5000; // Timeout in milliseconds (e.g., 5 seconds)


// === Timing ===
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 50; // ms

void setup() {
  Serial.begin(9600);
  Serial.println("Enter target position (in steps):");

  // Motor pins
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // Encoder pins
  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinCLK), readEncoder, CHANGE);
}

void loop() {
  // === Handle Serial Input ===
  if (Serial.available() > 0) {
  targetPosition = Serial.parseInt();
  Serial.readStringUntil('\n'); // clear buffer
  Serial.print("Target position set to: ");
  Serial.println(targetPosition);
  moveStartTime = millis(); // Start timeout timer
}

  //=== Run control loop at fixed interval ===
  unsigned long now = millis();
  if (now - lastControlTime >= controlInterval) {
    updateMotor();
    lastControlTime = now;
  }
}

// === Encoder ISR ===
void readEncoder() {
  int clkState = digitalRead(pinCLK);
  int dtState = digitalRead(pinDT);

  if (clkState != lastCLK) {
    if (dtState == clkState) {
      encoderPos--;  // adjust this based on motor/encoder wiring
    } else {
      encoderPos++;
    }
  }
  lastCLK = clkState;
}


// === Motor Control Function ===
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
  static bool motorRunning = false;
  long currentPos;

  // Read encoder position atomically
  noInterrupts();
  currentPos = encoderPos;
  interrupts();

  long error = targetPosition - currentPos;
  int direction = (error > 0) ? 1 : (error < 0) ? -1 : 0;

  int speed = abs(error) * Kp;
  if (speed < minSpeed && error != 0) speed = minSpeed;
  if (speed > maxSpeed) speed = maxSpeed;

  unsigned long now = millis();

  // === Timeout condition ===
  if (motorRunning && (now - moveStartTime > moveTimeout)) {
    moveMotor(0, 0); // Stop motor
    Serial.println("⚠️ Timeout: Motor did not reach target in time.");
    motorRunning = false;
    return;
  }

  if (abs(error) < 1) {
    moveMotor(0, 0); // Stop
    if (motorRunning) {
      Serial.println("Motor stopped. Target position reached.");
      motorRunning = false;
    }
  } else {
    moveMotor(direction, speed);
    if (!motorRunning) {
      moveStartTime = now; // reset timer when movement starts
    }
    motorRunning = true;

    // Only print when motor is active
    Serial.print("Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Current Pos: ");
    Serial.print(currentPos);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}



// // === Control Loop ===
// void updateMotor() {
//   static bool motorRunning = false; // track motor state
//   long currentPos;

//   // Read encoder position atomically
//   noInterrupts();
//   currentPos = encoderPos;
//   interrupts();

//   long error = targetPosition - currentPos;
//   int direction = (error > 0) ? 1 : (error < 0) ? -1 : 0;

//   int speed = abs(error) * Kp;
//   if (speed < minSpeed && error != 0) speed = minSpeed;
//   if (speed > maxSpeed) speed = maxSpeed;

//   if (abs(error) < 1) {
//     moveMotor(0, 0); // Stop motor
//     if (motorRunning) {
//       Serial.println("Motor stopped. Target position reached.");
//       motorRunning = false;
//     }
//   } else {
//     moveMotor(direction, speed);
//     motorRunning = true;

//     // Only print when motor is active
//     Serial.print("Target: ");
//     Serial.print(targetPosition);
//     Serial.print(" | Current Pos: ");
//     Serial.print(currentPos);
//     Serial.print(" | Error: ");
//     Serial.print(error);
//     Serial.print(" | Speed: ");
//     Serial.println(speed);
//   }
// }

