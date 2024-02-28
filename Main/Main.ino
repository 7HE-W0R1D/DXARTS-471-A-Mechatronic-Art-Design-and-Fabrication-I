#include <Servo.h>


Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

#define feedbackPin A0
#define servoMinDegree 0
#define servoMaxDegree 30

int pos = 0;    // variable to store the servo position
int analog_input;
int analog_mapped;

// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int tolerance = 2; // max feedback measurement error


void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.write(minPos);
  minDegrees = minPos;
  delay(2000); // make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  
  // Move to the maximum position and record the feedback value
  servo.write(maxPos);
  maxDegrees = maxPos;
  delay(2000); // make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
}

int reportDegrees()
{
  analog_input = analogRead(feedbackPin);
  analog_mapped = map(analog_input, minFeedback, maxFeedback, minDegrees, maxDegrees);
  Serial.print("Analog Input: ");
  Serial.print(analog_input);
  Serial.print("  Mapped Degrees: ");
  Serial.println(analog_mapped);
  return analog_mapped;
}

#include <avr/interrupt.h>

// Pin definitions
#define motor1_Pin1 5          // L298N motor pin 1
#define motor1_Pin2 6          // L298N motor pin 2
#define motor2_Pin1 9          // L298N motor pin 3
#define motor2_Pin2 10          // L298N motor pin 4
#define buttonPin1 4         // Button for forward
#define buttonPin2 5         // Button for backward
#define photoresistorPin1 A0 // Photoresistor 1 pin
#define photoresistorPin2 A1 // Photoresistor 2 pin

// Threshold values for photoresistors
const int threshold1 = 500; // Threshold for photoresistor 1
const int threshold2 = 500; // Threshold for photoresistor 2

volatile bool photoresistor1ReachedThreshold = false;
volatile bool photoresistor2ReachedThreshold = false;

class Motor {
    private:
        int motorPin1;
        int motorPin2;

    public:
        Motor(int pin1, int pin2) {
            motorPin1 = pin1;
            motorPin2 = pin2;
            pinMode(motorPin1, OUTPUT);
            pinMode(motorPin2, OUTPUT);
        }

        void forward(int speed, int duration) {
            analogWrite(motorPin1, speed);
            digitalWrite(motorPin2, LOW);
            delay(duration);
            stopMotor();
        }

        void backward(int speed, int duration) {
            digitalWrite(motorPin1, LOW);
            analogWrite(motorPin2, speed);
            delay(duration);
            stopMotor();
        }

        void stopMotor() {
            digitalWrite(motorPin1, LOW);
            digitalWrite(motorPin2, LOW);
        }
};

Motor motor1(motor1_Pin1, motor1_Pin2);

void setup() {
    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(photoresistorPin1, INPUT);
    pinMode(photoresistorPin2, INPUT);

    // Enable interrupts for photoresistor pins
    // attachInterrupt(digitalPinToInterrupt(photoresistorPin1), photoresistor1Interrupt, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(photoresistorPin2), photoresistor2Interrupt, CHANGE);
}

void loop() {

    int motorSpeed = 150;
    motor1.forward(motorSpeed, 2000);
    delay(5000);
    motor1.backward(motorSpeed, 2000);


    // // Check if button 2 is pressed
    // if (digitalRead(buttonPin2) == LOW) {
    //     motor2.backward();
    // }

}

void photoresistor1Interrupt() {
    if (analogRead(photoresistorPin1) >= threshold1) {
        photoresistor1ReachedThreshold = true;
    }
}                               

// void photoresistor2Interrupt() {
//     if (analogRead(photoresistorPin2) >= threshold2) {
//         photoresistor2ReachedThreshold = true;
//     }
// }