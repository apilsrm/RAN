#define INA 9
#define INB 10
#define PWM 5
#define EN A0
#define pinCLK 2
#define pinDT 3
const int UpperLimit = 7;
const int LowerLimit = 8;

volatile long encoderPos = 0;
volatile int lastCLK = 0;
int direction = 0;
volatile long stopEncoderPos = 0; // To track position when stop is triggered
volatile unsigned long stopTime = 0; // To track time when stop is triggered
volatile bool stopping = false; // Flag to indicate stopping process

void moveMotor(int direction, int speed);
void readEncoder();
void brakeMotor();

void setup() {
  Serial.begin(115200); // Increased baud rate for faster serial output
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  pinMode(UpperLimit, INPUT_PULLUP);
  pinMode(LowerLimit, INPUT_PULLUP);

  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinCLK), readEncoder, CHANGE);
}

void loop() {
  static int lastUpperRead = HIGH;
  static int lastLowerRead = HIGH;
  int UpperRead = digitalRead(UpperLimit);
  int LowerRead = digitalRead(LowerLimit);

  // Detect button press and initiate stop
  if (UpperRead == LOW && lastUpperRead == HIGH) {
    direction = -1;
    stopping = false; // Reset stopping flag
  } else if (LowerRead == LOW && lastLowerRead == HIGH) {
    direction = 1;
    stopping = false; // Reset stopping flag
  } else if ((UpperRead == HIGH && lastUpperRead == LOW) || (LowerRead == HIGH && lastLowerRead == LOW)) {
    // Button released, stop motor
    direction = 0;
    stopping = true;
    stopEncoderPos = encoderPos; // Record position at stop
    stopTime = micros(); // Record time at stop
    brakeMotor(); // Apply braking
  }

  lastUpperRead = UpperRead;
  lastLowerRead = LowerRead;

  // Only move motor if not stopping
  if (!stopping) {
    moveMotor(direction, 80);
  }

  // Log steps and time after stop
  if (stopping && (micros() - stopTime > 50000)) { // Wait 50ms to ensure motor stops
    Serial.print("Stopped at encoder position: ");
    Serial.println(encoderPos);
    Serial.print("Steps after stop: ");
    Serial.println(encoderPos - stopEncoderPos);
    Serial.print("Time after stop (us): ");
    Serial.println(micros() - stopTime);
    stopping = false; // Reset for next cycle
  }
}

void moveMotor(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else if (direction == -1) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH); // Short brake
    speed = 0;
  }
  analogWrite(PWM, speed);
}

void brakeMotor() {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, HIGH); // Short brake: both inputs HIGH
  analogWrite(PWM, 0); // Ensure no PWM signal
}

void readEncoder() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros();

  if (interruptTime - lastInterruptTime > 10) { // 10us debounce
    int clkState = digitalRead(pinCLK);
    int dtState = digitalRead(pinDT);
    if (clkState != lastCLK) {
      encoderPos += (dtState == clkState) ? -1 : 1;
    }
    lastCLK = clkState;
    lastInterruptTime = interruptTime;
  }
  // Print position only when needed to reduce serial overhead
  if (!stopping) {
    Serial.print("Position: ");
    Serial.println(encoderPos);
  }
}