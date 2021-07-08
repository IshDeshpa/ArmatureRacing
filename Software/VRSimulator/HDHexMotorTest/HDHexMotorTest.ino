#include <Servo.h>

int pwmPin = 2;
int powerPin = 26;

int pulseWidthUs = 2000;
int pdMs =  1/50 * 1000;

int pulseWidthMax = 2000;
int pulseWidthNeutral = 1500;
int pulseWidthMin = 1000;

Servo motor;


void setup() {
  // put your setup code here, to run once:
  pinMode(powerPin, OUTPUT);
  //pinMode(pwmPin, OUTPUT);

  motor.attach(pwmPin);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(powerPin, HIGH);
  //analogWrite(pwmPin, (pulseWidthUs - pulseWidthNeutral + 40)/(pulseWidthMax - pulseWidthNeutral + 40) * 255);
  //Serial.println((pulseWidthUs - pulseWidthNeutral + 40)/(pulseWidthMax - pulseWidthNeutral + 40) * 255);
//  digitalWrite(pwmPin, HIGH);
//  delay(pulseWidthMs);
//  digitalWrite(pwmPin, LOW);
//  delay(pdMs - pulseWidthMs);
  motor.writeMicroseconds(1500);
}
