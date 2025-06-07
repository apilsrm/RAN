// Pin definitions for VNH2SP30 motor driver
#define INA 9    // Motor direction pin A
#define INB 10    // Motor direction pin B
#define PWM 5    // PWM pin for speed control
#define EN A0    // Enable pin for motor driver

// Pin definitions for rotary encoder
#define pinCLK 2 // Encoder CLK pin (interrupt-capable)
#define pinDT 3  // Encoder DT pin

// Variables to track encoder and motor state
volatile long encoderPos = 0;  // Current encoder position
int lastCLK;                   // Last CLK state for encoder
unsigned long lastInterruptTime = 0; // For debouncing
const unsigned long debounceDelay = 15; // 10ms debounce delay
const long testDuration = 2000; // Duration of each test in ms

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Motor and Encoder Manual Test");
  Serial.println("Enter PWM value (0-255):");

  // Set motor driver pins as outputs
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN, OUTPUT);
  
  // Enable motor driver
  digitalWrite(EN, HIGH);

  // Set encoder pins as inputs with pull-up resistors
  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT, INPUT_PULLUP);

  // Read initial CLK state
  lastCLK = digitalRead(pinCLK);

  // Attach interrupt for encoder CLK pin
  attachInterrupt(digitalPinToInterrupt(pinCLK), readEncoder, CHANGE);
}

// Interrupt service routine for encoder with debouncing
void readEncoder() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime < debounceDelay) return; // Ignore if too soon
  lastInterruptTime = currentTime;

  int currentCLK = digitalRead(pinCLK);
  if (currentCLK != lastCLK) {
    if (digitalRead(pinDT) != currentCLK) {
      encoderPos++;
      Serial.print("encoderPos");
      Serial.println(encoderPos);
    } else {
      encoderPos--;
      Serial.print("encoderPos");
      Serial.println(encoderPos);
    }
  }
  lastCLK = currentCLK;
}

// Function to control motor direction and speed
void moveMotor(int direction, int speed) {
  if (direction == -1) { // Reverse
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (direction == 1) { // Forward
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else { // Stop with active braking
    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH); // Both HIGH for braking
    speed = 0;
  }
  analogWrite(PWM, speed);
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    int pwm = Serial.parseInt(); // Read PWM value
    Serial.readStringUntil('\n'); // Clear buffer

    // Validate PWM value
    if (pwm >= 0 && pwm <= 255) {
      // Reset encoder position
      encoderPos = 0;
      
      // Start motor at specified PWM value (forward direction)
      Serial.print("Testing PWM: ");
      Serial.print(pwm);
      Serial.print("...");
      moveMotor(1, pwm);
      
      // Run for test duration
      unsigned long startTime = millis();
      while (millis() - startTime < testDuration) {
        // Allow interrupts to update encoderPos
      }
      
      // Stop motor
      moveMotor(0, 0);
      
      // Print results
      Serial.print("\tEncoder Counts: ");
      Serial.print(encoderPos);
      Serial.print("\tDuration: ");
      Serial.println(testDuration);
      
      // Prompt for next input
      Serial.println("Enter next PWM value (0-255):");
    } else {
      Serial.println("Invalid PWM value. Enter a number between 0 and 255:");
    }
  }
}