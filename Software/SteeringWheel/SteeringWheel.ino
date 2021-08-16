#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "track.h"
#include <SoftwareSerial.h>

#define TFT_CS        10
#define TFT_RST        8
#define TFT_DC         7
#define TFT_BL         9

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
#define GOLD     0xE5ED
#define DARKRED  0xA000
#define ORANGE   0xFC45

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/*
  Temperature of motor, battery
  Battery charge percent
  Warning popups for main points of failure
  Speed in mph
  Charging indicator
  Odometer
  Drive mode (selected by dial)
  Confirm/OK button (confirms selection of drive modes)
  Switch menu button
  Pit lane limiter (speed limiter)
  Neutral button
*/

int sideBoxWidth = 90;
int centerBoxWidth = 320 - 2*sideBoxWidth;
int roundRectWidth = 80;
int roundRectHeight = 50;

int charWidth = 5;
int charHeight = 8;

int prevCharge = -1;
int charge = 100;
int prevSpeedMph = -1;
int speedMph = 125;

char prevPrimaryGear = 'Z';
char primaryGear = 'P';
char prevSecondaryGear = 'Z';
char secondaryGear = 'T';

int invTitleWidth = String("INV").length()*charWidth*2;
int prevInvTemp = -1;
int invTemp = 0;

int tmTitleWidth = String("TM").length()*charWidth*2;
int prevTmTemp = -1;
int tmTemp = 75;

int batTitleWidth = String("BAT").length()*charWidth*2;
int prevBatTemp = -1;
int batTemp = 30;

String readString = "";

#define STATE          6
#define TX_BLU         3
#define RX_BLU         2

// Address of Steering Wheel Bluetooth Module: 14:3:50943



void drawRGBBitmapComp(int16_t x, int16_t y, const uint16_t bitmap[], const int16_t trackSize, const int16_t startColor,
                                  int16_t w, int16_t h) {
  tft.startWrite();
  /*for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if((bool)pgm_read_byte(&bitmap[j * w + i]) == 1){
        tft.writePixel(x + i, y, WHITE);
      }
      else{
        tft.writePixel(x + i, y, BLACK);
      }
    }
  }*/

  int16_t currPixelX = x;
  int16_t currPixelY = y;
  int16_t currColor = startColor==0?0x0000:0xFFFF;
  for(int16_t i = 0; i < trackSize; i++){
    uint16_t cnt = pgm_read_word(&bitmap[i]);
    for(int16_t j = 0; j < cnt; j++){
      tft.writePixel(currPixelX, currPixelY, currColor);
      if(currPixelX < x+w-1){
        currPixelX++;  
      }
      else{
        currPixelY++;
        currPixelX = x;
      }
    }
    currColor = currColor==0x0000?0xFFFF:0x0000;
  }
  tft.endWrite();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Init
  tft.init(240,320);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(true);

  //Boxes
  tft.fillRect(0, 190, sideBoxWidth, 50, GREEN);
  tft.fillRect(sideBoxWidth, 190, centerBoxWidth, 50, WHITE);
  tft.fillRect(centerBoxWidth + sideBoxWidth, 190, sideBoxWidth, 50, GOLD);
  tft.drawRoundRect(320 - roundRectWidth - 10, 10, roundRectWidth, roundRectHeight, 11, GREEN);
  tft.drawRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight + 9, roundRectWidth, roundRectHeight, 11, GREEN);
  tft.drawRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight*2 + 18, roundRectWidth, roundRectHeight, 11, GREEN);

  //Labels
  tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - invTitleWidth/2, 5);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("INV");
  
  tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - tmTitleWidth/2, 5+roundRectHeight+9);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("TM");

  tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - batTitleWidth/2, 5+roundRectHeight*2+18);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("BAT");

  //Track map
  drawRGBBitmapComp(10, 10, track, trackSize, startColor, width, height);

  //Bluetooth
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) {
   // Serial.println(readString);  //so you can see the captured string
    charge = readString.toInt();  //convert readString into a number

  }

  readString=""; //empty for next input
  
  //Charge
  if(charge != prevCharge){
    prevCharge = charge;
    if(charge < 16.6){
      tft.fillRect(0, 190, sideBoxWidth, 50, RED);
    }
    else if(charge < 33.3){
      tft.fillRect(0, 190, sideBoxWidth, 50, YELLOW);
    }
    else if(charge < 50){
      tft.fillRect(0, 190, sideBoxWidth, 50, ORANGE);
    }
    else{
      tft.fillRect(0, 190, sideBoxWidth, 50, GREEN);
    }
 
    int chargeTextWidth = (String(charge).length() + 1)*charWidth*4;
    int percentWidth = String("%").length()*charWidth*2 - 6;
    
    tft.setCursor(sideBoxWidth/2 - (chargeTextWidth + percentWidth)/2, 200);
    tft.setTextColor(BLACK);
    tft.setTextSize(4);
    tft.print(String(charge));

    tft.setCursor(sideBoxWidth/2 - (chargeTextWidth + percentWidth)/2 + chargeTextWidth - 6, 210);
    tft.setTextSize(2);
    tft.print("%");
  }

  //Speed
  if(speedMph != prevSpeedMph){
    prevSpeedMph = speedMph;
    tft.fillRect(sideBoxWidth, 190, centerBoxWidth, 50, WHITE);

    int speedTextWidth = String(speedMph).length()*charWidth*6;
    int mphWidth =  String("mph").length()*charWidth*1 + 6;
    
    tft.setCursor(sideBoxWidth + centerBoxWidth/2 - ((speedTextWidth+mphWidth)/2), 195);
    tft.setTextColor(BLACK, WHITE);
    tft.setTextSize(6);
    tft.print(String(speedMph));

    tft.setTextSize(1);
    tft.setCursor(sideBoxWidth + centerBoxWidth/2 - ((speedTextWidth+mphWidth)/2) + speedTextWidth + 6, 195);
    tft.print("mph");
  }
  

  //Gears
  if(primaryGear != prevPrimaryGear || secondaryGear != prevSecondaryGear){
    prevPrimaryGear = primaryGear;
    prevSecondaryGear = secondaryGear;
    tft.fillRect(centerBoxWidth + sideBoxWidth, 190, sideBoxWidth, 50, GOLD);
    
    int primaryGearWidth = charWidth*6;
    int secondaryGearWidth = charWidth*4;
    
    tft.drawChar(centerBoxWidth + sideBoxWidth*3/2 - (primaryGearWidth + secondaryGearWidth + 5)/2, 195, primaryGear, BLACK, GOLD, 6);
    tft.drawChar(centerBoxWidth + sideBoxWidth*3/2 - (primaryGearWidth + secondaryGearWidth + 5)/2 + primaryGearWidth + 5, 205, secondaryGear, DARKRED, GOLD, 4);
  }
  
  
  
  
  
  if(invTemp != prevInvTemp){
    prevInvTemp = invTemp;
    tft.fillRoundRect(320 - roundRectWidth - 10, 10, roundRectWidth, roundRectHeight, 11, BLACK);
    tft.drawRoundRect(320 - roundRectWidth - 10, 10, roundRectWidth, roundRectHeight, 11, GREEN);
    
    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - invTitleWidth/2, 5);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("INV");

    int invTempWidth = String(invTemp).length()*charWidth*4;
    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - invTempWidth/2, 20);
    tft.setTextColor(GOLD);
    tft.setTextSize(4);
    tft.print(invTemp);
  }

  
  if(tmTemp != prevTmTemp){
    prevTmTemp = tmTemp;
    tft.fillRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight + 9, roundRectWidth, roundRectHeight, 11, BLACK);
    tft.drawRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight + 9, roundRectWidth, roundRectHeight, 11, GREEN);

    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - tmTitleWidth/2, 5+roundRectHeight+9);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("TM");
    
    int tmTempWidth = String(tmTemp).length()*charWidth*4;
    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - tmTempWidth/2, 20+roundRectHeight+9);
    tft.setTextColor(GOLD);
    tft.setTextSize(4);
    tft.print(tmTemp);
  }

  
  if(batTemp != prevBatTemp){
    prevBatTemp = batTemp; 
    tft.fillRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight*2 + 18, roundRectWidth, roundRectHeight, 11, BLACK);
    tft.drawRoundRect(320 - roundRectWidth - 10, 10 + roundRectHeight*2 + 18, roundRectWidth, roundRectHeight, 11, GREEN);

    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - batTitleWidth/2, 5+roundRectHeight*2+18);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("BAT");
    
    int batTempWidth = String(batTemp).length()*charWidth*4;
    tft.setCursor(320 - roundRectWidth - 10 + roundRectWidth/2 - batTempWidth/2, 20+roundRectHeight*2+18);
    tft.setTextColor(GOLD);
    tft.setTextSize(4);
    tft.print(batTemp);
  }
}
