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
long targetPos = 0;            // Target position
int lastCLK;                   // Last CLK state for encoder
unsigned long lastInterruptTime = 0; // For debouncing
const unsigned long debounceDelay = 10; // 10ms debounce delay

// Proportional control parameters
const float Kp = 5.0;          // Proportional gain
const int maxMotorSpeed = 200; // Maximum PWM speed
const int minMotorSpeed = 50;  // Minimum PWM speed to prevent stalling
int lastpm = 0;

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

// Interrupt service routine for encoder with debouncing
void readEncoder() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime < debounceDelay) return; // Ignore if too soon
  lastInterruptTime = currentTime;

  int currentCLK = digitalRead(pinCLK);
  if (currentCLK != lastCLK) {
    if (digitalRead(pinDT) != currentCLK) {
      encoderPos++;
      Serial.print(encoderPos);
      Serial.print(" ");

     
    } else {
      encoderPos--;
      Serial.println(encoderPos);
      
    }
  }
  lastCLK = currentCLK;
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Motor and Encoder Position Test with Proportional Control");
  Serial.println("Enter target position (e.g., 100 or -100):");

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

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    targetPos = Serial.parseInt(); // Read target position
    Serial.readStringUntil('\n'); // Clear buffer

    // Reset encoder position
    encoderPos = 0;
    
    // Start test
    Serial.print("Moving to position: ");
    Serial.println(targetPos);
    Serial.print("Initial encoder position: ");
    Serial.println(encoderPos);

    // Move motor until target is reached
    while (true) {
      // Calculate error
      long error = targetPos - encoderPos;
      
      // Determine direction
      int direction = 0;
      if (error > 0) {
        direction = 1; // Forward
      } else if (error < 0) {
        direction = -1; // Reverse
      } else {
        moveMotor(0, 0); // Stop motor
        break; // Exit loop
      }

      // Calculate PWM using proportional control
      int pwm = abs(error) * Kp;
      if (pwm < minMotorSpeed) pwm = minMotorSpeed;
      if (pwm > maxMotorSpeed) pwm = maxMotorSpeed;

     
    
          
      // Move motor
      moveMotor(direction, pwm);
    }
    Serial.print(pwm)

    // Print results
    Serial.print("Final encoder position: ");
    Serial.println(encoderPos);
    Serial.print("Error (target - actual): ");
    Serial.println(targetPos - encoderPos);
    Serial.println("Test complete. Enter next target position:");
  }
}