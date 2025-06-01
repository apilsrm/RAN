#include <AccelStepper.h>

// Define stepper motor connections and interface type
//connect 1winding 2wire to a+ ang A- and same for 2ndwinding 
// For a driver with STEP and DIR pins, use AccelStepper::DRIVER (or 1)
// Pin 5 = STEP pin, Pin 3 = DIR pin (adjust pins as per your wiring)
AccelStepper stepper(AccelStepper::DRIVER, 5, 3);

void setup() {
  // Set max speed (steps per second)
  stepper.setMaxSpeed(200);
  // Set acceleration (steps per second squared)
  stepper.setAcceleration(1000);
  // Set current position to zero
  stepper.setCurrentPosition(0);
}

void loop() {
  // Move to position 4000 steps forward
  stepper.moveTo(10000);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  delay(1000); // Wait for 1 second

  // Move back to position 0 (starting point)
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  delay(1000); // Wait for 1 second
}