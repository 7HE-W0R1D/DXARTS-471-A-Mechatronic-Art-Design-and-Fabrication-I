// Pin definitions
#define forwardPin 5
#define backwardPin 6

void motorControl(bool forward, int speed);

void setup() {
  // Set the pin modes
  pinMode(10, OUTPUT);
  analogWrite(10, 255);
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
}

void loop() {
    // Move the motor forward at full speed
    motorControl(true, 255);
    delay(2000);
    // Move the motor backward at full speed
    motorControl(false, 255);
    delay(2000);
    // Stop the motor
    motorControl(true, 0);
    delay(2000);
    // Move the motor forward at half speed
    motorControl(true, 127);
    delay(2000);
    // Move the motor backward at half speed
    motorControl(false, 127);
    delay(2000);
    // Stop the motor
    motorControl(true, 0);
    delay(2000);
}

void motorControl(bool forward=true, int speed=255) {
  if(forward) {
    digitalWrite(backwardPin, LOW);
    analogWrite(forwardPin, speed);
  }
  else{
    digitalWrite(forwardPin, LOW);
    analogWrite(backwardPin, speed);
  }
}