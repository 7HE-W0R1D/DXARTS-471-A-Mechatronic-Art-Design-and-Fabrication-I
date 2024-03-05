// HC-SR501 PIR Sensor with pin 3 connected

#define pirPin 3 // PIR sensor output pin

volatile int motion = LOW; // Variable to store motion state

void setup() {
  pinMode(pirPin, INPUT); // Set the PIR sensor pin as input
  attachInterrupt(digitalPinToInterrupt(pirPin), motionDetected, RISING); // Attach interrupt to the PIR sensor pin
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Do nothing in the loop, the interrupt will handle motion detection
}

void motionDetected() {
  // motion = digitalRead(pirPin); // Read the PIR sensor output

  // if (motion == HIGH) {
  //   Serial.println("Motion detected!"); // Print a message if motion is detected
  // } else {
  //   Serial.println("No motion detected."); // Print a message if no motion is detected
  // }

  Serial.println("Motion detected!"); // Print a message if motion is detected 
}
