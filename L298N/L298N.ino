#include <avr/interrupt.h>

// Pin definitions
#define motor1_Pin1 5          // L298N motor pin 1
#define motor1_Pin2 6          // L298N motor pin 2
#define motor2_Pin1 9          // L298N motor pin 3
#define motor2_Pin2 10          // L298N motor pin 4
#define forwardStopSwitch 2         // Button for forward
#define backwardStopSwitch 3         // Button for backward


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

        void reset() {
            forward(255, 8000);
            stopMotor();
        }
};


Motor motor1(motor1_Pin1, motor1_Pin2);

void stopMotor1() {
    motor1.stopMotor();
}

void setup() {
    pinMode(forwardStopSwitch, INPUT);
    pinMode(backwardStopSwitch, INPUT_PULLUP);
    // Enable interrupts for stop switch pins
    attachInterrupt(digitalPinToInterrupt(forwardStopSwitch), stopMotor1, RISING);
    // attachInterrupt(digitalPinToInterrupt(backwardStopSwitch), stopMotor1, FALLING);

    motor1.reset();
}

void loop() {

    int motorSpeed = 250;
    motor1.forward(motorSpeed, 2000);
    delay(2000);
    motor1.backward(motorSpeed, 2000);
    delay(2000);

    // // Check if button 2 is pressed
    // if (digitalRead(buttonPin2) == LOW) {
    //     motor2.backward();
    // }

}