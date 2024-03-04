// HC-SR501 PIR Sensor with pin 7 connected

const int pirPin = 7; // PIR sensor output pin

void setup() {
  pinMode(pirPin, INPUT); // Set the PIR sensor pin as input
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  int motion = digitalRead(pirPin); // Read the PIR sensor output

  if (motion == HIGH) {
    Serial.println("Motion detected!"); // Print a message if motion is detected
  } else {
    Serial.println("No motion detected."); // Print a message if no motion is detected
  }

  delay(1000); // Delay for 1 second
}
