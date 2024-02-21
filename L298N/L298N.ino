#include <avr/interrupt.h>

// Pin definitions
#define motor1_Pin1 2          // L298N motor pin 1
#define motor1_Pin2 3          // L298N motor pin 2
#define motor2_Pin1 6          // L298N motor pin 3
#define motor2_Pin2 7          // L298N motor pin 4
#define buttonPin1 4         // Button for forward
#define buttonPin2 5         // Button for backward
#define photoresistorPin1 A0 // Photoresistor 1 pin
#define photoresistorPin2 A1 // Photoresistor 2 pin

#define motorPin1 2          // L298N motor pin 1
#define motorPin2 3          // L298N motor pin 2

// Threshold values for photoresistors
const int threshold1 = 500; // Threshold for photoresistor 1
const int threshold2 = 500; // Threshold for photoresistor 2

volatile bool photoresistor1ReachedThreshold = false;
volatile bool photoresistor2ReachedThreshold = false;

// Motor control functions
void forward(int motor = 1)
{
    if (motor == 1)
    {
        digitalWrite(motor1_Pin1, HIGH);
        digitalWrite(motor1_Pin2, LOW);
    }
    else
    {
        digitalWrite(motor2_Pin1, HIGH);
        digitalWrite(motor2_Pin2, LOW);
    }
    delay(3000);
    stopMotor();
}

void backward(int motor = 1)
{
    if (motor == 1)
    {
        digitalWrite(motor1_Pin1, LOW);
        digitalWrite(motor1_Pin2, HIGH);
    }
    else
    {
        digitalWrite(motor2_Pin1, LOW);
        digitalWrite(motor2_Pin2, HIGH);
    }
    delay(3000);
    stopMotor();
}

void stopMotor()
{
    digitalWrite(motor1_Pin1, LOW);
    digitalWrite(motor1_Pin2, LOW);
    digitalWrite(motor2_Pin1, LOW);
    digitalWrite(motor2_Pin2, LOW);
}

void setup()
{
    // Set pin modes
    pinMode(motor1_Pin1, OUTPUT);
    pinMode(motor1_Pin2, OUTPUT);
    pinMode(motor2_Pin1, OUTPUT);
    pinMode(motor2_Pin2, OUTPUT);
    
    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(photoresistorPin1, INPUT);
    pinMode(photoresistorPin2, INPUT);

    // Enable interrupts for photoresistor pins
    attachInterrupt(digitalPinToInterrupt(photoresistorPin1), photoresistor1Interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(photoresistorPin2), photoresistor2Interrupt, CHANGE);
}

void loop()
{
    // Check if button 1 is pressed
    if (digitalRead(buttonPin1) == LOW)
    {
        forward();
    }

    // Check if button 2 is pressed
    if (digitalRead(buttonPin2) == LOW)
    {
        backward();
    }

    // Check if photoresistor 1 reaches threshold
    if (photoresistor1ReachedThreshold)
    {
        stopMotor();
        photoresistor1ReachedThreshold = false;
    }

    // Check if photoresistor 2 reaches threshold
    if (photoresistor2ReachedThreshold)
    {
        stopMotor();
        photoresistor2ReachedThreshold = false;
    }
}

void photoresistor1Interrupt()
{
    if (analogRead(photoresistorPin1) >= threshold1)
    {
        photoresistor1ReachedThreshold = true;
    }
}

void photoresistor2Interrupt()
{
    if (analogRead(photoresistorPin2) >= threshold2)
    {
        photoresistor2ReachedThreshold = true;
    }
}
