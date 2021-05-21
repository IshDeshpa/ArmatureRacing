```
//#include <FFBDescriptor.h>
//#include <Joystick.h>

//#include <config.h>
//#include <order.h>

#include "Joystick.h"
#include "RotaryEncoder.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_GAMEPAD, 0, 0,
  true, true, true, false, false, false,
  false, false, false, false, false);

//Joystick_ Joystick();

const int powerPin1 = 22;
const int powerPin2 = 24;

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

//Force Feedback
/*Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};*/

RotaryEncoder encoder(steeringOutputA, steeringOutputB, RotaryEncoder::LatchMode::TWO03);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  
  Joystick.setYAxisRange(0, maxValueAccel - minValueAccel);
  Joystick.setZAxisRange(0, maxValueBrake - minValueBrake);
  Joystick.setXAxisRange(0, encoderRes/2*6);

  //mygains[0].totalGain = 100;//0-100
  //mygains[0].springGain = 100;//0-100

  pinMode(powerPin1, OUTPUT);
  pinMode(powerPin2, OUTPUT);

  pinMode(steeringOutputA, INPUT);
  pinMode(steeringOutputB, INPUT);

  encoder.setPosition(0);

  //Joystick.setGains(mygains);
  Joystick.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(powerPin1, HIGH);
  digitalWrite(powerPin2, HIGH);
  
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
    //Serial.println(newPos);
    pos = newPos;
  }

  //myeffectparams[0].springMaxPosition = encoderRes/2*6;
  //myeffectparams[0].springPosition = newPos;

  Joystick.setXAxis(newPos);

  //Joystick.setEffectParams(myeffectparams);
  //Joystick.getForce(forces);

  //SerialUSB.println("Force: " + forces[0]);
  //SerialUSB.println(newPos);
  
  //delay(1);
}
```
