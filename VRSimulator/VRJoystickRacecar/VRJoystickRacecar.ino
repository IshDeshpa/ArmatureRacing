//#include <FFBDescriptor.h>
//#include <Joystick.h>

//#include <config.h>
//#include <order.h>

#include "Joystick.h"
#include "RotaryEncoder.h"

#include <Servo.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_GAMEPAD, 0, 0,
  true, true, true, true, false, false,
  false, false, false, false, false);

//Joystick_ Joystick();

const int powerPin1 = 22;
const int powerPin2 = 24;
const int powerPin3 = 26;

const int accelPedalPin = A0;
const int minValueAccel = 255;
const int maxValueAccel = 700;
int accelerator = 0;

const int brakePedalPin = A1;
const int minValueBrake = 170;
const int maxValueBrake = 310;
int brake = 0;

const int steeringOutputA = 48;
const int steeringOutputB = 50;
const int encoderRes = 300;

const int pwmPin = 2;
const int pulseWidthMax = 2000;
const int pulseWidthNeutral = 1500;
const int pulseWidthMin = 1000;
const double motorGain = 0.5;

const int dead = 30;

int distFromCenter = 0;
double distFromCenterConstrained = 0;
int motorSpeed = 0;


Servo motor;

//Force Feedback
RotaryEncoder encoder(steeringOutputA, steeringOutputB, RotaryEncoder::LatchMode::TWO03);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  Joystick.setYAxisRange(0, maxValueAccel - minValueAccel);
  Joystick.setZAxisRange(0, maxValueBrake - minValueBrake);
  Joystick.setXAxisRange(0, encoderRes/2*6);

  //Joystick.setRxAxisRange(-encoderRes/2*3, encoderRes/2*3);

  pinMode(powerPin1, OUTPUT);
  pinMode(powerPin2, OUTPUT);
  pinMode(powerPin3, OUTPUT);

  pinMode(steeringOutputA, INPUT);
  pinMode(steeringOutputB, INPUT);

  encoder.setPosition(0);

  //motor.attach(pwmPin);
  distFromCenter = (encoderRes/2*6)/2;
  
  Joystick.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(powerPin1, HIGH);
  digitalWrite(powerPin2, HIGH);
  digitalWrite(powerPin3, HIGH);
  
  accelerator = analogRead(accelPedalPin);
  if(accelerator < minValueAccel){
    accelerator = minValueAccel;
  }
  else if(accelerator > maxValueAccel){
    accelerator = maxValueAccel;  
  }
  accelerator -= minValueAccel;

  Joystick.setYAxis(accelerator);

  brake = analogRead(brakePedalPin);
  
  if(brake < minValueBrake){
    brake = minValueBrake;
  }
  else if(accelerator > maxValueBrake){
    brake = maxValueBrake;  
  }
  brake -= minValueBrake;

  Joystick.setZAxis(brake);

  static int pos = 0;
  encoder.tick();

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    pos = newPos;
  }
  
  Joystick.setXAxis(newPos);

  //distFromCenter = encoderRes/2*3 - newPos;
  //distFromCenterConstrained = distFromCenter/(encoderRes/2*3.0)*1.0;

  //Serial.println(distFromCenter);
  //Serial.println(distFromCenterConstrained);

  /*if(newPos < encoderRes/2*3 + dead && newPos > encoderRes/2*3 - dead){
    motor.writeMicroseconds(1500);
  }
  else if(newPos > encoderRes/2*3 + dead){
    motorSpeed = 1250;
    motor.writeMicroseconds(motorSpeed);   
  }
  else if(newPos < encoderRes/2*3 - dead){
    motorSpeed = 1750;
    motor.writeMicroseconds(motorSpeed);  
  }*/
  
  
  
  //motorSpeed = map(pow(distFromCenterConstrained, 2), -1, 1, pulseWidthMin, pulseWidthMax);
  
  
  //Joystick.setRxAxis(distFromCenter);
}
