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

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
    
  calibrate(myservo, feedbackPin, servoMinDegree, servoMaxDegree);
}

void loop() {
  servoMimicHddFullRead(myservo);
  reportDegrees();
  delay(1000);

  for (int i = 0; i < 10; i++) {
    servoMimicHddRandomRead(myservo);
    reportDegrees();
    delay(750);
  }

  delay(100000);
}

void moveServoBackAndForth(Servo servo, int minPos, int maxPos, int delayTime) {
  for (int pos = minPos; pos <= maxPos; pos++) {
    servo.write(pos);
    delay(delayTime);
  }
  for (int pos = maxPos; pos >= minPos; pos--) {
    servo.write(pos);
    delay(delayTime);
  }
}

void servoMimicHddFullRead(Servo servo) {
  moveServoBackAndForth(servo, servoMinDegree, servoMaxDegree, 40);
}

void servoMimicHddRandomRead(Servo servo) {
  int randomPos = random(servoMinDegree, servoMaxDegree);
  Serial.println(randomPos);
  servo.write(randomPos);
}