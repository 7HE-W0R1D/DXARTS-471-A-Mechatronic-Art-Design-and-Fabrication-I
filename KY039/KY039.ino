// KY039 Sensor Pin Configuration
const int KY039_PIN = A0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the sensor value
  int sensorValue = analogRead(KY039_PIN);

  // Print the sensor value to the serial monitor
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);

  // Add a delay before the next reading
  delay(1000);
}
