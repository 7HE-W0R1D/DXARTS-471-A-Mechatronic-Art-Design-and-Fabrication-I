#include <Servo.h>

// Buzzer Setup Code
#define buzzer 11

// Scanning Head Setup Code
#define scanningLED 13 // LED pin

void lightingEffect()
{

    float counter = 1;
    int multiplier = 500;
    float temp = 2 / counter;
    int delayTime = multiplier * temp;

    while (delayTime > 30)
    {
        digitalWrite(scanningLED, HIGH);
        tone(buzzer, 1000 + 1000 * temp);
        delay(delayTime);
        digitalWrite(scanningLED, LOW);
        noTone(buzzer);
        delay(delayTime);
        counter++;
        temp = 2 / counter;
        delayTime = multiplier * (temp);
        // Serial.println(temp);
    }
    digitalWrite(scanningLED, LOW);
    delay(1250);
    tone(buzzer, 1000);
    digitalWrite(scanningLED, HIGH);
    delay(1000);
    noTone(buzzer);
}

// Servo Setup Code
Servo readPinServo; // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo handServo;

#define feedbackPin A0
#define servoMinDegree 0
#define servoMaxDegree 30

int pos = 0; // variable to store the servo position
int analog_input;
int analog_mapped;

// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;

// void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
// {
//     // Move to the minimum position and record the feedback value
//     servo.write(minPos);
//     minDegrees = minPos;
//     delay(2000); // make sure it has time to get there and settle
//     minFeedback = analogRead(analogPin);

//     // Move to the maximum position and record the feedback value
//     servo.write(maxPos);
//     maxDegrees = maxPos;
//     delay(2000); // make sure it has time to get there and settle
//     maxFeedback = analogRead(analogPin);
// }

void moveServoBackAndForth(Servo servo, int minPos, int maxPos, int delayTime)
{
    for (int pos = minPos; pos <= maxPos; pos++)
    {
        servo.write(pos);
        delay(delayTime);
    }
    for (int pos = maxPos; pos >= minPos; pos--)
    {
        servo.write(pos);
        delay(delayTime);
    }
}

void servoMimicHddFullRead(Servo servo)
{
    moveServoBackAndForth(servo, servoMinDegree, servoMaxDegree, 40);
}


void servoMimicHddRandomRead(Servo servo)
{
    int randomPos = random(servoMinDegree, servoMaxDegree);
    // Serial.println(randomPos);
    servo.write(randomPos);
}

/**
 * @brief Moves the hand servo in a waving motion.
 */
void servoWavingHand()
{
    for (int i = 0; i < 2; i++)
    {
        handServo.write(0);
        delay(500);
        handServo.write(120);
        delay(500);
    }
}

// void handServoIdle()
// {
//     int minPos = 0; // Minimum position of the servo
//     int maxPos = 180; // Maximum position of the servo
//     int delayTime = 250; // Delay time between movements

//     for (int i = 0; i < 5; i++) // Repeat the motion 5 times
//     {            
//         if (isHuman)
//         {
//             motionDetected();
//             isHuman = false;
//         };

//         int startPos = random(minPos, maxPos); // Generate a random start position
//         int randomPos = startPos + random(10, 30); // Generate a random position
//         handServo.write(startPos); // Move the hand servo to the random position
//         delay(delayTime);
//         handServo.write(randomPos); // Move the hand servo to the start position
//         delay(2000);
//     }
// }


// Motor Setup Code
#include <avr/interrupt.h>

// Pin definitions
#define motor1_Pin1 5        // L298N motor pin 1
#define motor1_Pin2 6        // L298N motor pin 2
#define forwardStopSwitch 2  // Button for forward
#define backwardStopSwitch 3 // Button for backward

class Motor
{
private:
    int motorPin1;
    int motorPin2;

public:
    Motor(int pin1, int pin2)
    {
        motorPin1 = pin1;
        motorPin2 = pin2;
        pinMode(motorPin1, OUTPUT);
        pinMode(motorPin2, OUTPUT);
    }

    void forward(int speed, int duration)
    {
        analogWrite(motorPin1, speed);
        digitalWrite(motorPin2, LOW);
        delay(duration);
        stopMotor();
    }

    void backward(int speed, int duration)
    {
        digitalWrite(motorPin1, LOW);
        analogWrite(motorPin2, speed);
        delay(duration);
        stopMotor();
    }

    void stopMotor()
    {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
    }

    void reset()
    {
        forward(255, 4000);
        stopMotor();
    }
};

Motor motor1(motor1_Pin1, motor1_Pin2);

void stopMotor1()
{
    motor1.stopMotor();
}

void offerDisc()
{
    int motorSpeed = 150;
    motor1.forward(motorSpeed, 2000);
    delay(1000);

    for (int i = 0; i < 5; i++)
    {
        motor1.backward(motorSpeed, 500);
        delay(100);
        motor1.forward(motorSpeed, 500);
        delay(100);
    }
    delay(3000);
    motor1.backward(motorSpeed, 2000);
    delay(1000);
}

// PIR Sensor Setup Code
#define pirPin 3 // PIR sensor output pin


bool isHuman = false;

void routineFunction() {
    int seed = random(0,3);
    if (seed == 0) {
        servoMimicHddFullRead(readPinServo);
    }
    else if (seed == 1) {
        int randomReads = random(10, 20);
        for (int i = 0; i < randomReads; i++)
        {

            servoMimicHddRandomRead(readPinServo);
            delay(200);
            digitalWrite(scanningLED, HIGH);
            tone(buzzer, 1000);
            delay(50);
            digitalWrite(scanningLED, LOW);
            noTone(buzzer);
            // reportDegrees();
            delay(650);
            if (isHuman)
            {
                motionDetected();
                isHuman = false;
            };
        }
    }
    else {
    int minPos = 0; // Minimum position of the servo
    int maxPos = 180; // Maximum position of the servo
    int delayTime = 250; // Delay time between movements

    for (int i = 0; i < 5; i++) // Repeat the motion 5 times
    {            
        if (isHuman)
        {
            motionDetected();
            isHuman = false;
        };

        int startPos = random(minPos, maxPos); // Generate a random start position
        int randomPos = startPos + random(10, 30); // Generate a random position
        handServo.write(startPos); // Move the hand servo to the random position
        delay(delayTime);
        handServo.write(randomPos); // Move the hand servo to the start position
        delay(2000);
    }
    }

}

void setup()
{
    pinMode(buzzer, OUTPUT);

    pinMode(pirPin, INPUT); // Set the PIR sensor pin as input
    // attachInterrupt(digitalPinToInterrupt(pirPin), motionDetected, RISING); // Attach interrupt to the PIR sensor pin
    attachInterrupt(digitalPinToInterrupt(pirPin), humanPresent, RISING); // Attach interrupt to the PIR sensor pin

    readPinServo.attach(9); // attaches the servo on pin 9 to the servo object
    handServo.attach(10);   // attaches the servo on pin 10 to the servo object

    pinMode(scanningLED, OUTPUT); // Set the LED pin as output

    Serial.begin(9600);

    motor1.reset();
    // calibrate(readPinServo, feedbackPin, servoMinDegree, servoMaxDegree);
    
}

void loop()
{
    if (isHuman)
    {
        motionDetected();
        isHuman = false;
    };

    digitalWrite(scanningLED, HIGH);
    if (isHuman)
    {
        motionDetected();
        isHuman = false;
    };

    routineFunction();
}

void humanPresent()
{
    isHuman = true;
}

void motionDetected()
{
    Serial.println("Motion detected!"); // Print a message if motion is detected
    digitalWrite(scanningLED, LOW);
    readPinServo.write(servoMinDegree);
    servoWavingHand();
    lightingEffect();
    offerDisc();
    delay(2000);
}
