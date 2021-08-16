#include <SoftwareSerial.h>

#define STATE          6
#define TX_BLU         3
#define RX_BLU         2

SoftwareSerial HC05(RX_BLU, TX_BLU);

char c = ' ';

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HC05.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  //from bluetooth to Terminal. 
 if (HC05.available()){
   c = HC05.read();
   Serial.write(c);
 }
 //from termial to bluetooth 
 if (Serial.available()){
   HC05.write(Serial.read());
 }
}
