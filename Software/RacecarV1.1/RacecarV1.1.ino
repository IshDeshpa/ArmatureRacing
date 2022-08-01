// TODO clean up the charge_activate vs charge_deactivate code, deal with analog inputs for accelerator pedal, separation of charge and battery vars, difference between msgs10ms and
// ncroutine??

HardwareSerial *serial;

#include <due_can.h>  
#include <due_wire.h>
#include <Wire_EEPROM.h> 
#include <Chrono.h>

#define Ignition            25  // Global Enable signal
#define MC                  2   // Motor controller relay
#define Ign                 3   // Ignition relay
#define Failsafe            4   // Failsafe relay
#define FailsafeChg         5   // Failsafe Charge Relay
#define SysMain1            6   // System Main 1 Relay (signals relay inside battery)
#define SysMain2            7   // System Main 2 Relay (signals relay inside battery)
#define PreChg              8   // Precharge Relay (signals relay inside battery)
#define BatOff              9   // Provides power from small 12V battery until DCDC Converter enables
#define EVSystemActivation  10  // EVSE plug in signal from the Onboard Charger, active LOW
#define DCDCTemp            11  // Temperature reading from DCDC Converter, active LOW
#define DCDCAct             12  // Activation signal to DCDC Converter, active HIGH
#define DCDCTwelveVCont     13  // Voltage selection signal to DCDC Converter, active HIGH

// Change these depending on which printouts you want
#define PrintInverter
#define PrintBattery
#define PrintConverter
#define PrintCharger
#define PrintRelays
#define PrintPedals

// Millisecond timers
Chrono timer_Frames10 = Chrono();   // For CAN
Chrono timer_Frames100 = Chrono();  // For CAN
Chrono timer_hv = Chrono();
Chrono timer_inouts = Chrono();
Chrono timer_chg = Chrono();
Chrono timer_serial = Chrono();
//Chrono timer_3sec = Chrono();

// DB stands for Debounce, or input stabilization
Chrono timer_ev_sys_act_db = Chrono();
Chrono timer_ignition_db = Chrono();
Chrono timer_chg_flag_db = Chrono();

CAN_FRAME outFrame;   // A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    // Structure to keep inbound inFrames

//Universally important thingies
bool Pch_Flag = false;
bool HV_Flag = false;
bool global_enable = false;
String readString;

//------------------------------------------------------------------------------DCDC CONVERTER STUFF
bool dcdc_enable = false;
bool twelve_v_cont = false;
volatile int pwm_value = 0;
volatile int prev_time = 0;

//------------------------------------------------------------------------------INVERTER STUFF
struct InverterStatus
{
  uint16_t voltage = 0;
  int16_t speed = 0;
  int8_t inverter_temperature = 0;
  int8_t motor_temperature = 0;
  bool error_state = false;
} inverter_status;

int inv_volts_local;
int inv_speed_local;
int16_t final_torque_request = 0;

#define INVERTER_BITS_PER_VOLT 2
#define INVERTER_BITS_PER_RPM 2

//------------------------------------------------------------------------------PEDAL STUFF
const int accelPedalPin = A0;
const int minValueAccel = 151;
const int maxValueAccel = 959;
int accelerator = 0;

/*
const int brakePedalPin = A1;
const int minValueBrake = 170;
const int maxValueBrake = 310;
int brake = 0;
*/

//------------------------------------------------------------------------------CHARGING STUFF
bool charge_enable = false;

bool charge_activation = false;
bool charge_deactivation = false;
bool charge_constant = false;

uint16_t current_charge_power = 0x64;

uint16_t counter_1f2 = 0;

uint16_t charger_status = 0;
bool NC_relay_status = false;
bool QC_relay_status = false;

uint16_t AC_voltage_charger = 0;

bool chargerInPort380 = false;
bool chargerCurrent380 = false;

double LBC_Max_Power_1DC = 0;
int LB_Failsafe_Status = 0;
int LB_Output_Power_Limit_Reason = 0;
int charge_power_status_LBC = 0;
int LB_Battery_Voltage = 0;
int LB_Current = 0;
double OBC_Max_Power_380 = 0;
uint16_t LB_SOC = 0;

uint16_t BatteryTempC = 0;
int J1772_Current_Limiter = 0;

bool LBC_Charge_Flag = false;

uint16_t remain_capacity_gids = 0;
uint16_t new_full_capacity_wh = 0;
uint16_t dash_bars_capacity = 0;
uint16_t dash_bars_charge = 0;
uint16_t time_remaining_charge = 0;
uint16_t charge_complete_flag = 0;

uint16_t MPRUN_380 = -1;

//------------------------------------------------------------------------------------setup------------------------------------------------------------------------------------
void setup() {
  Can0.begin(CAN_BPS_500K);   // Inverter/LBattery/OBC CAN
  Can0.watchFor();

  //Can1.begin(CAN_BPS_500K); // Currently unused bus
  //Can1.watchFor();
  
  pinMode(Ignition, INPUT_PULLUP);
  pinMode(MC, OUTPUT);
  pinMode(Ign, OUTPUT);
  pinMode(Failsafe, OUTPUT);
  pinMode(FailsafeChg, OUTPUT);
  pinMode(SysMain1, OUTPUT);
  pinMode(SysMain2, OUTPUT);
  pinMode(PreChg, OUTPUT);
  pinMode(BatOff, OUTPUT);
  pinMode(EVSystemActivation, INPUT_PULLUP);
  pinMode(DCDCTemp, INPUT_PULLUP);
  pinMode(DCDCAct, OUTPUT);
  pinMode(DCDCTwelveVCont, OUTPUT);
  
  // Set all relays/mosfets to the off state
  digitalWrite(MC, HIGH);
  digitalWrite(Ign, HIGH);
  digitalWrite(Failsafe, HIGH);
  digitalWrite(FailsafeChg, HIGH);
  digitalWrite(SysMain1, HIGH);
  digitalWrite(SysMain2, HIGH);
  digitalWrite(PreChg, HIGH);
  digitalWrite(DCDCAct, LOW);
  digitalWrite(DCDCTwelveVCont, LOW);

  // Except for the Battery Off relay, which provides Arduino+LBat+OBC startup power
  digitalWrite(BatOff, LOW);

  // A way to read the PWM signals
  attachInterrupt(digitalPinToInterrupt(DCDCTemp), fall, FALLING);

  //USB Serial
  //Serial.begin(115200);
  //serial = &Serial;
  
  //Bluetooth Serial
  Serial1.begin(38400);
  serial = &Serial1;

  while(!serial){}

  // I don't think this is needed...
  //timer_chg.restart();
}

//------------------------------------------------------------------------------------loop------------------------------------------------------------------------------------
void loop() {
  
  //------------------------------------------------------------------------------READ AND WRITE NON-CAN SIGNALS
  if (timer_inouts.hasPassed(5))// Includes pwm output stuff
  {
    timer_inouts.restart();
    CheckIO();
  }
  
  //------------------------------------------------------------------------------HV CONTROL
  if (timer_hv.hasPassed(500))
  {
    timer_hv.restart();
    HVControl();
    //digitalWrite(MC, LOW);
  }
  
  /*if(digitalRead(EVSystemActivation) == HIGH){
    serial->println("AAAAAAAAAAAAAAAAAA");  
  }*/
  
  //------------------------------------------------------------------------------SERIAL TORQUE REQUEST
  while (serial->available()) {
    char c = serial->read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) {
   // serial->println(readString);  //so you can see the captured string
    final_torque_request = readString.toInt();  //convert readString into a number

  }

  readString=""; //empty for next input
  
  //------------------------------------------------------------------------------CAN
  CheckCAN();

  if (global_enable && !charge_enable)
  {
    Msgs100ms();  //fire the 100ms can messages
    Msgs10ms();   //fire the 10ms can messages
  }
  else if (charge_enable && !global_enable)
  {
    //serial->println("charge enabled");
    NCRoutine();
    Msgs100ms();
  }
  //------------------------------------------------------------------------------SERIAL READOUTS
  if (timer_serial.hasPassed(5000))
  {
    serial->println("---------------");
    
    if (global_enable) { serial->println("Enabled"); }
    else { serial->println("Disabled"); }

    #ifdef PrintBattery
      serial->print("LB Battery Voltage: ");
      serial->println(LB_Battery_Voltage);
      serial->print("LB_Current: ");
      serial->println(LB_Current);
      serial->print("Battery Temp C: ");
      serial->println(BatteryTempC);
    #endif
    
    #ifdef PrintInverter
      serial->print("Inverter Voltage: ");
      serial->println(inv_volts_local);
      serial->print("Motor Speed: ");
      serial->println(inv_speed_local);
    #endif
    
    #ifdef PrintConverter
      serial->print("DCDC Converter Enabled: ");
      serial->println(dcdc_enable);
      serial->print("DCDC Converter Temperature Percentage: ");
      if (pwm_value <= 4 && pwm_value >= 2.5) { serial->println(map(pwm_value, 250, 400, 0, 100)); }
      else { serial->println("read error :("); }
    #endif
    
    #ifdef PrintCharger
      serial->print("Full Capacity Wh: ");
      serial->println(new_full_capacity_wh);
      serial->print("LB SOC: ");
      serial->println(LB_SOC);
      serial->print("OBC status: ");
      serial->println(charger_status);
      serial->print("LBC charge flag: ");
      serial->println(LBC_Charge_Flag);
      serial->print("LB Failsafe status: ");
      serial->println(LB_Failsafe_Status);
      serial->print("LBC Charge Power Status: ");
      serial->println(charge_power_status_LBC);
      serial->print("LB_Output_Power_Limit_Reason: ");
      serial->println(LB_Output_Power_Limit_Reason);
      serial->print("J1772_Current_Limiter: ");
      serial->println(J1772_Current_Limiter);
      serial->print("NC Relay: ");
      serial->println(NC_relay_status);
      serial->print("Current gid capacity: ");
      serial->println(remain_capacity_gids);
      serial->print("Current charge power: ");
      serial->println(current_charge_power);
      serial->print("Time remaining for charge: ");
      serial->println(time_remaining_charge);
      serial->print("Charge complete flag: ");
      serial->println(charge_complete_flag);
    #endif
    
    #ifdef PrintRelays
      serial->print("MC Relay: ");
      serial->println(!digitalRead(MC));
      serial->print("Ign Relay: ");
      serial->println(!digitalRead(Ign));
      serial->print("Failsafe Relay: ");
      serial->println(!digitalRead(Failsafe));
      serial->print("FailsafeChg Relay: ");
      serial->println(!digitalRead(FailsafeChg));
      serial->print("SysMain1 Relay: ");
      serial->println(!digitalRead(SysMain1));
      serial->print("SysMain2 Relay: ");
      serial->println(!digitalRead(SysMain2));
      serial->print("PreChg Relay: ");
      serial->println(!digitalRead(PreChg));
      serial->print("BatOff Relay: ");
      serial->println(!digitalRead(BatOff));
    #endif

    #ifdef PrintPedals
      serial->print("Accelerator: ");
      serial->println(accelerator);
      //serial->print("Brake: ");
      //serial->println(brake);
    #endif
    
    serial->println("---------------");
    timer_serial.restart();
  }
}

//------------------------------------------------------------------------------------CheckIO------------------------------------------------------------------------------------
void CheckIO()
{
  //------------------------------------------------------------------------------IGNITION SWITCH
  bool prev_ge = global_enable;
  global_enable = !digitalRead(Ignition); // High and low equal to 1 and 0, or true and false. The Ignition switch is HIGH when off.
  if (prev_ge != global_enable)
  {
    if (timer_ignition_db.hasPassed(50))
    {
      serial->print("GLOBAL ENABLE ");
      serial->println(global_enable);
      timer_ignition_db.restart();
    }
    else
    {
      global_enable = prev_ge;
    }
  }

  //------------------------------------------------------------------------------PEDALS
  accelerator = analogRead(accelPedalPin);
  accelerator = constrain(accelerator, minValueAccel, maxValueAccel);
  accelerator = map(accelerator, minValueAccel, maxValueAccel, 0, 1023);
  
  //brake = analogRead(brakePedalPin);
  //brake = constrain(brake, minValueBrake, maxValueBrake, 0, maxValueBrake - minValueBrake);
  //brake = map(brake, minValueBrake, maxValueBrake, 0, maxValueBrake - minValueBrake);
  

  //------------------------------------------------------------------------------EV SYS ACT
  if (!digitalRead(EVSystemActivation) && timer_ev_sys_act_db.hasPassed(2000) && !chargerInPort380)
  {
    serial->println("EVSE DETECTED, CHARGE ENABLED");
    charge_enable = true;
    timer_ev_sys_act_db.restart();
  }

  //------------------------------------------------------------------------------ CONVERTER
  if (HV_Flag)// Active tasks
  {
    if (!dcdc_enable)
    {
      dcdc_enable = true;
      digitalWrite(DCDCAct, HIGH);
      serial->println("DCDC Activate");
    }
    
    // Output the 13V signal to the DCDC Converter, 25-75 ms high for 13-15V
    if (millis()%125 < 25)
    {
      if(!twelve_v_cont) {digitalWrite(DCDCTwelveVCont, HIGH);}
      twelve_v_cont = true;
    }
    else if (twelve_v_cont)
    {
      digitalWrite(DCDCTwelveVCont, LOW);
      twelve_v_cont = false;
    }
  }
  else if (dcdc_enable)// Deactivate
  {
    digitalWrite(DCDCAct, LOW);
    digitalWrite(DCDCTwelveVCont, LOW);
    twelve_v_cont = false;
    dcdc_enable = false;
    serial->println("DCDC Deactivate");
  }
}

//------------------------------------------------------------------------------------DCDCTempPWM------------------------------------------------------------------------------------
void fall()
{
  attachInterrupt(digitalPinToInterrupt(DCDCTemp), rise, RISING);
  prev_time = micros();
  //serial->println("Fall triggered");
}

void rise()
{
  attachInterrupt(digitalPinToInterrupt(DCDCTemp), fall, FALLING);
  pwm_value = micros()-prev_time;
  //serial->println("Rise triggered");
}

//------------------------------------------------------------------------------------HVControl------------------------------------------------------------------------------------
void HVControl()
{  
  inv_volts_local = (inverter_status.voltage / INVERTER_BITS_PER_VOLT);
  inv_speed_local = (inverter_status.speed/ INVERTER_BITS_PER_RPM);

  if(!charge_enable)
  {
    //------------------------------------------------------------------------------CAR IGNITION ON
    if (global_enable && !Pch_Flag)  //if terminal 15 is on and precharge not enabled
    {
      serial->println("MC/IGN/FS/Main2 RELAY ON");
      digitalWrite(MC, LOW);  //inverter power on
      digitalWrite(Ign, LOW);  //ignition relay on
      digitalWrite(Failsafe, LOW);
      digitalWrite(SysMain2, LOW);  //main contactor on (for 1 rail)
      digitalWrite(SysMain1, HIGH);
      
      if(inv_volts_local<200)
      {
        serial->println("Ignition ON");
        digitalWrite(PreChg, LOW);  //precharge on
        Pch_Flag=true;
      }
    }
    
    if (global_enable && !HV_Flag && Pch_Flag)  //using inverter measured hv for initial tests. Will use ISA derived voltage in final version.
    {
      serial->println("Precharging");
      if (inv_volts_local>340)
      {
        serial->println("Main1 RELAY ON, Pchg OFF");
        digitalWrite(SysMain1, LOW);  //main contactor on
        digitalWrite(PreChg, HIGH);
        HV_Flag=true;  //hv on flag
      }
    }
    
    //------------------------------------------------------------------------------CAR IGNITION OFF
    if (!global_enable && Pch_Flag)
    {
      serial->println("Ignition OFF");
      digitalWrite(PreChg, HIGH);  //precharge off
      digitalWrite(SysMain1, HIGH);  //main contactor off
      digitalWrite(SysMain2, HIGH);  //main contactor off
      digitalWrite(MC, HIGH);  //inverter power off
      digitalWrite(Ign, HIGH);  //ignition relay off
      digitalWrite(Failsafe, HIGH);
      digitalWrite(FailsafeChg, HIGH);
      Pch_Flag = false;
      HV_Flag = false;
    }
  }

  
  else
  {
    //------------------------------------------------------------------------------CHARGING ON
    if(!global_enable)
    {
      digitalWrite(MC, LOW);
      //digitalWrite(Ign, LOW);
      //serial->println("charge enabled");
      digitalWrite(Failsafe, LOW);
      digitalWrite(FailsafeChg, LOW);
      
      if(charge_activation || charge_constant)
      {
        if (!Pch_Flag)  // if terminal 15 is on and precharge not enabled
        {
          digitalWrite(SysMain2, LOW);  // main contactor on (for 1 rail)
          digitalWrite(SysMain1, HIGH);
          
          if(inv_volts_local<200)
          {
            serial->println("Precharge ON");
            digitalWrite(PreChg, LOW);  // precharge on
            Pch_Flag=true;
          }
        }
        if (!HV_Flag && Pch_Flag)  // using inverter measured hv for initial tests. Will use ISA derived voltage in final version.
        {
          serial->println("Precharging");
          if (inv_volts_local>340)
          {
            serial->println("Main1 RELAY ON, Pchg OFF");
            digitalWrite(SysMain1, LOW);  // main contactor on
            digitalWrite(PreChg, HIGH);
            HV_Flag=true;  //hv on flag
          }
        }
      }
    }
    
    //------------------------------------------------------------------------------CHARGING END
    else
    {
      serial->println("Global enable on - CHARGE DISABLE");
      digitalWrite(PreChg, HIGH);  //precharge off
      digitalWrite(SysMain1, HIGH);  //main contactor off
      digitalWrite(SysMain2, HIGH);  //main contactor off
      digitalWrite(MC, HIGH);  //inverter power off
      digitalWrite(Ign, HIGH);  //ignition relay on
      digitalWrite(Failsafe, HIGH);
      digitalWrite(FailsafeChg, HIGH);
      Pch_Flag = false;
      HV_Flag = false;
      inv_volts_local = 0;
    }
  }
}

//------------------------------------------------------------------------------------CheckCAN------------------------------------------------------------------------------------
void CheckCAN(){
  //serial->println(inFrame.id, HEX);
  if(Can0.available())
  {
    Can0.read(inFrame);
    //serial->println(inFrame.id, HEX);

    if(inFrame.id == 0x55b && inFrame.length == 8){
      LB_SOC = (inFrame.data.bytes[0] << 2) | ((inFrame.data.bytes[1] & 0b11000000) >> 6);
    }

    if(inFrame.id == 0x1db && inFrame.length == 8){
      
      LB_Failsafe_Status = (inFrame.data.bytes[1] & 0b111);
      LB_Battery_Voltage = ((inFrame.data.bytes[2] << 2) | ((inFrame.data.bytes[3] & 0b11000000) >> 6))/2;
      LB_Current = ((inFrame.data.bytes[0] << 3) | ((inFrame.data.bytes[3] & 0b11100000) >> 5))/2;
    }
    
    if(inFrame.id == 0x1da && inFrame.length == 8){
    //  last_received_from_inverter_timestamp = millis();
  
      inverter_status.voltage = ((uint16_t)inFrame.data.bytes[0] << 2) |
          (inFrame.data.bytes[1] >> 6);
  
      int16_t parsed_speed = ((uint16_t)inFrame.data.bytes[4] << 8) |
          (uint16_t)inFrame.data.bytes[5];
      inverter_status.speed = (parsed_speed == 0x7fff ? 0 : parsed_speed);
  
      inverter_status.error_state = (inFrame.data.bytes[6] & 0xb0) != 0x00;
    }

    uint16_t checksumTotal = 0;

    if(inFrame.id == 0x5BF && inFrame.length == 8)
    {
      //serial->println(inFrame.data.bytes[2]*.5);
      
      for(int i=0; i<7; i++)
      {
        checksumTotal += (inFrame.data.bytes[i] & 0xF0) >> 4;
        checksumTotal += inFrame.data.bytes[i] & 0x0F;
      }
      
      checksumTotal += (inFrame.data.bytes[7] & 0xF0) >> 4;
      checksumTotal -= 0x01;
      checksumTotal = checksumTotal & 0x0F;

      //serial->println(checksumTotal);
      //serial->println(inFrame.data.bytes[7] & 0x0F);
      
      if((inFrame.data.bytes[7] & 0x0F) == checksumTotal)
      {
        charger_status = inFrame.data.bytes[4];
        J1772_Current_Limiter = inFrame.data.bytes[2]/2;
      }
    }

    if(inFrame.id == 0x1DB && inFrame.length == 8){
      if(timer_chg_flag_db.hasPassed(1000)){
        LBC_Charge_Flag = (inFrame.data.bytes[3] & 0b00010000) >> 4;
        timer_chg_flag_db.restart();
      }
    }

    if(inFrame.id == 0x380 && inFrame.length == 8){
      //serial->print("NC Relay Status:");
      if((inFrame.data.bytes[4] & 0b01000000)>>6 == 1){
        NC_relay_status = true;
      }
      else{
        NC_relay_status = false;
      }
      //serial->println(NC_relay_status);
      //serial->print("QC Relay Status:");
      if((inFrame.data.bytes[4] & 0b00100000)>>5 == 1){
        QC_relay_status = true;
      }
      else{
        QC_relay_status = false;
      }
      //serial->println(QC_relay_status);

      if(MPRUN_380 == -1){
        MPRUN_380 = (inFrame.data.bytes[7] & 0b11110000) >> 4;
      }
      else{
        if(MPRUN_380 == (inFrame.data.bytes[7] & 0b11110000) >> 4){
          serial->println("FROZEN DATA ERROR");
          //TODO: Something to handle error here
        }
        else{
          MPRUN_380 = (inFrame.data.bytes[7] & 0b11110000) >> 4;
          //serial->println((inFrame.data.bytes[7] & 11110000) >> 4, HEX);
        }
      }

      // Charger in port
      if((inFrame.data.bytes[2] & 0b00001000)>>3 == 1){ chargerInPort380 = true; }
      else{ chargerInPort380 = false; }

      // Charger current
      //serial->println(inFrame.data.bytes[4]);
      if((inFrame.data.bytes[4] & 0b00010000) >> 4 == 1){ chargerCurrent380 = true; }// Current detected
      else{ chargerCurrent380 = false; }// No current

      // Idk
      OBC_Max_Power_380 = ((inFrame.data.bytes[2] & 0b00000001) << 8 | (inFrame.data.bytes[3]));

      // Wall voltage from EVSE
      AC_voltage_charger = ((inFrame.data.bytes[5] & 0b00000111) << 6) | ((inFrame.data.bytes[6] & 0b11111100) >> 2);
      //serial->print("AC Voltage Into Charger: ");
      //serial->println(AC_voltage_charger);
    }

    // Max Power for Charging from LBC
    if(inFrame.id == 0x1DC && inFrame.length == 8)
    {
      LBC_Max_Power_1DC = (((inFrame.data.bytes[2] & 0b00000111) << 6) | ((inFrame.data.bytes[3] & 0b11111100) >> 2));
      charge_power_status_LBC = (inFrame.data.bytes[3] & 0b11);
      //serial->print("LBC Power x1DC: ");
      //serial->println(LBC_Max_Power_1DC);
      //serial->println(inFrame.data.bytes[2], BIN);

      /*serial->print("Byte 2: ");
      serial->println(inFrame.data.bytes[2] & 00001111, BIN);
      serial->print("Byte 3: ");
      serial->println(inFrame.data.bytes[3] & 11111100, BIN);
      serial->print("Concatenated: ");
      serial->println(((inFrame.data.bytes[2] & 00001111) << 6) | ((inFrame.data.bytes[3] & 11111100) >> 2), BIN);*/
    }

    // Capacity
    if(inFrame.id == 0x5BC && inFrame.length == 8){
      remain_capacity_gids = (inFrame.data.bytes[0] << 2) | ((inFrame.data.bytes[1] & 0b11000000) >> 6);
      new_full_capacity_wh = ((((inFrame.data.bytes[0] & 0b00111111) << 4) | ((inFrame.data.bytes[1] & 0b11110000) >> 4))) * 80 + 250;
      dash_bars_capacity = ((inFrame.data.bytes[4] & 0b00000001) == 1)?(inFrame.data.bytes[2] & 0b00001111):dash_bars_capacity;
      dash_bars_charge = ((inFrame.data.bytes[4] & 0b00000001) == 1)?dash_bars_charge:(inFrame.data.bytes[2] & 0b00001111);
      time_remaining_charge = ((inFrame.data.bytes[6] & 00011111) << 8) | inFrame.data.bytes[7];
      charge_complete_flag = (inFrame.data.bytes[5] & 00000100) >> 2;
      BatteryTempC = inFrame.data.bytes[3] - 40;
      LB_Output_Power_Limit_Reason = (inFrame.data.bytes[5] & 0b11100000);
    
      //serial->print(inFrame.data.bytes[4] * 0b00000001);
      //serial->print(": ");
      //serial->println(inFrame.data.bytes[2] & 0b00001111);
      //serial->println(dash_bars_capacity);
        
    }

//    if(inFrame.id == 0x5C0 && inFrame.length == 8){
//      int mux = (inFrame.data.bytes[0] & 0b11000000) >> 6;
//      if(mux == 2){
//        BatteryTempC = ((inFrame.data.bytes[2] & 0b11111110) >> 1) - 40;
//      }
//      
//    }
    
    /*if(inFrame.id == 0x55a && inFrame.length == 8){
     // last_received_from_inverter_timestamp = millis();
  
      inverter_status.inverter_temperature = fahrenheit_to_celsius(inFrame.data.bytes[2]);
      inverter_status.motor_temperature = fahrenheit_to_celsius(inFrame.data.bytes[1]);
      //serial->println(inverter_status.inverter_temperature);
    }*/

  }
  
  //------------------------------------------------------------------------------CHARGE ACTIVATE
  if(charge_enable && chargerInPort380 && charge_activation == false && 
  charge_constant == false && charge_deactivation == false)
  {
      serial->println("CHARGE ACTIVATE");
      charge_activation = true;
      timer_chg.restart();
  }

  //------------------------------------------------------------------------------CHARGE COMPLETE OR ABORT
  // || charger_status == 64 || !NC_relay_status || BatteryTempC > 60
  if((!chargerInPort380 || BatteryTempC > 60 || charge_complete_flag) && charge_enable && (charge_activation || charge_constant))
  {
    charge_activation = false;
    charge_constant = false;
    charge_deactivation = true;
  }
}

//------------------------------------------------------------------------------------Msgs10ms------------------------------------------------------------------------------------
void Msgs10ms()                       //10ms messages here
{
  if(timer_Frames10.hasPassed(10))
  {
    //serial->println("Sending msgs");
    timer_Frames10.restart();
    static uint8_t counter_11a_d6 = 0;
    static uint8_t counter_1d4 = 0;
    static uint8_t counter_1db = 0;
    
    outFrame.id = 0x11a;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 3 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 // No request
  
    outFrame.data.bytes[0] = 0x4E;
    //outFrame.data.bytes[0] = 0x01;
  
    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO
    outFrame.data.bytes[1] = 0x40;
  
    // Usually 0x00, sometimes 0x80 (LeafLogs), 0x04 seen by canmsgs
    outFrame.data.bytes[2] = 0x00;
  
    // Weird value at D3:4 that goes along with the counter
    // NOTE: Not actually needed, you can just send constant AA C0
    const static uint8_t weird_d34_values[4][2] = {
      {0xaa, 0xc0},
      {0x55, 0x00},
      {0x55, 0x40},
      {0xaa, 0x80},
    };
    outFrame.data.bytes[3] = weird_d34_values[counter_11a_d6][0];
    outFrame.data.bytes[4] = weird_d34_values[counter_11a_d6][1];
  
    // Always 0x00 (LeafLogs, canmsgs)
    outFrame.data.bytes[5] = 0x00;
  
    // A 2-bit counter
    outFrame.data.bytes[6] = counter_11a_d6;
  
    counter_11a_d6++;
    if(counter_11a_d6 >= 4)
      counter_11a_d6 = 0;
  
    // Extra CRC
    nissan_crc(outFrame.data.bytes, 0x85);
  
    /*serial->print(F("Sending "));
    print_fancy_inFrame(inFrame);
    serial->println();*/
  
    Can0.sendFrame(outFrame);

    // Send target motor torque signal
    outFrame.id = 0x1d4;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 3 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 // No request

    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: F70700E0C74430D4

    // Usually F7, but can have values between 9A...F7 (gen1)
    outFrame.data.bytes[0] = 0xF7;
    // 2016: 6E
   // outFrame.data.bytes[0] = 0x6E;

    // Usually 07, but can have values between 07...51 (gen1)
    outFrame.data.bytes[1] = 0x07;
    // 2016: 6E
    //outFrame.data.bytes[1] = 0x6E;

    // Requested torque (signed 12-bit value + always 0x0 in low nibble)
    static int16_t last_logged_final_torque_request = 0;
    if(final_torque_request != last_logged_final_torque_request){
      last_logged_final_torque_request = final_torque_request;
      //log_print_timestamp();
      serial->print(F("Sending torque request "));
      serial->print(final_torque_request);
      serial->print(F(" (speed: "));
      serial->print(inverter_status.speed / INVERTER_BITS_PER_RPM);
      serial->print(F(" rpm)"));
      serial->print(inverter_status.voltage / INVERTER_BITS_PER_VOLT);
      serial->print(F(" Volts)"));
      serial->println();
    }

    //Check if within bounds of 12 bit number   
    if(final_torque_request >= -2048 && final_torque_request <= 2047){
      outFrame.data.bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) | // Adds 1 to the beginning if negative
          ((final_torque_request >> 4) // Equivalent to dividing by 2^4 (16), shifts bits left by 4
          & 0x7f); // 
      outFrame.data.bytes[3] = (final_torque_request << 4) & 0xf0;
    } else {
      outFrame.data.bytes[2] = 0x00;
      outFrame.data.bytes[3] = 0x00;
    }

    // MSB nibble: Runs through the sequence 0, 4, 8, C
    // LSB nibble: Precharge report (precedes actual precharge
    //             control)
    //   0: Discharging (5%)
    //   2: Precharge not started (1.4%)
    //   3: Precharging (0.4%)
    //   5: Starting discharge (3x10ms) (2.0%)
    //   7: Precharged (93%)
    outFrame.data.bytes[4] = 0x07 | (counter_1d4 << 6); // Bitshift counter left by 6 (11 -> 11000000, 10 -> 10000000)
    // First part is LSB nibble, counter controls MSB nibble
    
    //outFrame.data.bytes[4] = 0x02 | (counter_1d4 << 6);

    counter_1d4++;
    if(counter_1d4 >= 4)
      counter_1d4 = 0;

    //counter_1d4 is only ever 0, 1, 2, 3 and cycles for every loop (only ever takes up two bits)

    // MSB nibble:
    //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
    //      after the car has been shut off (6% total)
    //   4: Otherwise (94%)
    // LSB nibble:
    //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
    //      D (0.3%)
    //   2: Reverse gear related (13%)
    //   4: Forward gear related (21%)
    //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
    //outFrame.data.bytes[5] = 0x44;
    //outFrame.data.bytes[5] = 0x46;

    // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
    // 0x46 requires ~25 torque to start
    //outFrame.data.bytes[5] = 0x46;
    // 0x44 requires ~8 torque to start
    outFrame.data.bytes[5] = 0x44;

    // MSB nibble:
    //   In a drive cycle, this slowly changes between values (gen1):
    //     leaf_on_off.txt:
    //       5 7 3 2 0 1 3 7
    //     leaf_on_rev_off.txt:
    //       5 7 3 2 0 6
    //     leaf_on_Dx3.txt:
    //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
    //     leaf_on_stat_DRDRDR.txt:
    //       0 1 3 7
    //     leaf_on_Driveincircle_off.txt:
    //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7 
    //     leaf_on_wotind_off.txt:
    //       3 2 0 8 A B 3 7
    //     leaf_on_wotinr_off.txt:
    //       5 7 3 2 0 8 A B 3 7
    //     leaf_ac_charge.txt:
    //       4 6 E 6
    //   Possibly some kind of control flags, try to figure out
    //   using:
    //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq | ~/projects/leaf_tools/util/hex_to_ascii_binary.py
    //   2016:
    //     Has different values!
    // LSB nibble:
    //   0: Always (gen1)
    //   1:  (2016)

    // 2016 drive cycle:
    //   E0: to 0.15s
    //   E1: 2 messages
    //   61: to 2.06s (inverter is powered up and precharge
    //                 starts and completes during this)
    //   21: to 13.9s
    //   01: to 17.9s
    //   81: to 19.5s
    //   A1: to 26.8s
    //   21: to 31.0s
    //   01: to 33.9s
    //   81: to 48.8s
    //   A1: to 53.0s
    //   21: to 55.5s
    //   61: 2 messages
    //   60: to 55.9s
    //   E0: to end of capture (discharge starts during this)

    // This value has been chosen at the end of the hardest
    // acceleration in the wide-open-throttle pull, with full-ish
    // torque still being requested, in
    //   LeafLogs/leaf_on_wotind_off.txt
    //outFrame.data.bytes[6] = 0x00;

    // This value has been chosen for being seen most of the time
    // when, and before, applying throttle in the wide-open-throttle
    // pull, in
    //   LeafLogs/leaf_on_wotind_off.txt
    outFrame.data.bytes[6] = 0x30;    //brake applied heavilly.

    // Value chosen from a 2016 log
    //outFrame.data.bytes[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
   // outFrame.data.bytes[6] = 0x01;
    //byte 6 brake signal

    // Extra CRC
    nissan_crc(outFrame.data.bytes, 0x85);

    /*serial->print(F("Sending "));
    print_fancy_inFrame(inFrame);
    serial->println();*/

//    serial->print(outFrame.id);
//    serial->print(": ");
//    for(int i=0; i<8; i++){
//      serial->print(outFrame.data.bytes[i], HEX);
//      serial->print(" ");
//    }
//    serial->println();
    Can0.sendFrame(outFrame);

/*    //We need to send 0x1db here with voltage measured by inverter

    outFrame.id = 0x1db;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 3 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 //No request
    outFrame.data.bytes[0]=0x00;  
    outFrame.data.bytes[1]=0x00;
    outFrame.data.bytes[2]=0x00;
    outFrame.data.bytes[3]=0x00;
    outFrame.data.bytes[4]=0x00;
    outFrame.data.bytes[5]=0x00;
    outFrame.data.bytes[6]=counter_1db;
    outFrame.data.bytes[7]=0x00;


    counter_1db++;
    if(counter_1db >= 4)
      counter_1db = 0;

        

    Can0.sendFrame(outFrame);
*/
  }
}

//------------------------------------------------------------------------------------Msgs100ms------------------------------------------------------------------------------------
void Msgs100ms(){
  if(timer_Frames100.hasPassed(100))
  {
    timer_Frames100.restart();
  
    //  digitalWrite(led, !digitalRead(led)); //toggle led everytime we fire the 100ms messages.
    
    outFrame.id = 0x50b;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 8 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 //No request
    // Statistics from 2016 capture:
    //     10 00000000000000
    //     21 000002c0000000
    //    122 000000c0000000
    //    513 000006c0000000

    // Let's just send the most common one all the time
    // FIXME: This is a very sloppy implementation
    //  hex_to_data(outFrame.data.bytes, "00,00,06,c0,00,00,00");
    outFrame.data.bytes[0]=0x00;
    outFrame.data.bytes[1]=0x00;  
    outFrame.data.bytes[2]=0x06;
    outFrame.data.bytes[2]=0x00;
    outFrame.data.bytes[3]=0xc0;
    outFrame.data.bytes[4]=0x00;
    outFrame.data.bytes[5]=0x00;
    outFrame.data.bytes[6]=0x00;
    outFrame.data.bytes[7]=0x00;
  
    /*CONSOLE.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    CONSOLE.println();*/
//    serial->print(outFrame.id, HEX);
//    serial->print(": ");
//    for(int i=0; i<8; i++){
//      serial->print(outFrame.data.bytes[i], HEX);
//      serial->print(" ");
//    }
//    serial->println();
    Can0.sendFrame(outFrame); 
  
  }
}

//------------------------------------------------------------------------------------NCRoutine------------------------------------------------------------------------------------
void NCRoutine(){
  if(charge_activation){
    if(current_charge_power >= OBC_Max_Power_380 + 100){
    //if(current_charge_power >= 0x6B){
      charge_activation = false;
      charge_deactivation = false;
      charge_constant = true;
    }

    current_charge_power = timer_chg.elapsed()*0.124+100;
    
    /*serial->print(timer_chg.elapsed());
    serial->print(", ");
    serial->println(current_charge_power, HEX);*/
  }

  
  else if(charge_constant){
    current_charge_power = 100+OBC_Max_Power_380;
    //serial->print(current_charge_power);
    //serial->println(", CONSTANT");
  }

  //------------------------------------------------------------------------------CHARGE DEACTIVATE
  if(charge_deactivation)
  {
    current_charge_power = 0x64;
    outFrame.id = 0x1F2;
    outFrame.length = 8;
    outFrame.extended = 0;
    outFrame.rtr = 1;
  
    outFrame.data.bytes[0] = 0x30;
    outFrame.data.bytes[1] = current_charge_power;
    outFrame.data.bytes[2] = 0x20;
    outFrame.data.bytes[3] = 0x00;
    outFrame.data.bytes[4] = 0x00;
    outFrame.data.bytes[5] = 0x82;
    outFrame.data.bytes[6] = counter_1f2;
    counter_1f2++;
    if(counter_1f2 >= 4)
    {
      counter_1f2 = 0;  
    }
    
    outFrame.data.bytes[7] = 0x02;
    for(int i=0; i<7; i++)
    {
       outFrame.data.bytes[7] += outFrame.data.bytes[i]&0xF0;
       outFrame.data.bytes[7] += outFrame.data.bytes[i]&0x0F;
    }
    
    outFrame.data.bytes[7] = outFrame.data.bytes[7] & 0x0F;

    Can0.sendFrame(outFrame);

    serial->println("DEACTIVATE");

    charge_constant = false;
    charge_activation = false;  
    charge_deactivation = false;
    charge_enable = false;
  }

  
  if(timer_Frames10.hasPassed(10) && !charge_deactivation){
    timer_Frames10.restart();

    static uint8_t counter_1d4 = 0;
    
    outFrame.id = 0x1F2;
    outFrame.length = 8;
    outFrame.extended = 0;
    outFrame.rtr = 1;
  
    outFrame.data.bytes[0] = 0x30;
    outFrame.data.bytes[1] = current_charge_power;
    //outFrame.data.bytes[1] = 0x64;
    outFrame.data.bytes[2] = 0x20;
    outFrame.data.bytes[3] = 0x00;
    outFrame.data.bytes[4] = 0x00;
    outFrame.data.bytes[5] = 0x82;
    outFrame.data.bytes[6] = counter_1f2;
    counter_1f2++;
    if(counter_1f2 >= 4){
      counter_1f2 = 0;  
    }
    
    /*outFrame.data.bytes[7] = 0x02;
    for(int i=0; i<7; i++){
       outFrame.data.bytes[7] += outFrame.data.bytes[i]&0xF0;
       outFrame.data.bytes[7] += outFrame.data.bytes[i]&0x0F;
    }
    
    outFrame.data.bytes[7] = outFrame.data.bytes[7] & 0x0F;*/
    outFrame.data.bytes[7] = counter_1f2 + 0x0A;

//    serial->print(outFrame.id, HEX);
//    serial->print(": ");
//    for(int i=0; i<8; i++){
//      serial->print(outFrame.data.bytes[i], HEX);
//      serial->print(" ");
//    }
//    serial->println();
    Can0.sendFrame(outFrame);

    // Send target motor torque signal
    outFrame.id = 0x1d4;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 3 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 //No request

    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: F70700E0C74430D4

    // Usually F7, but can have values between 9A...F7 (gen1)
    outFrame.data.bytes[0] = 0xF7;
    // 2016: 6E
   // outFrame.data.bytes[0] = 0x6E;

    // Usually 07, but can have values between 07...51 (gen1)
    outFrame.data.bytes[1] = 0x07;
    // 2016: 6E
    //outFrame.data.bytes[1] = 0x6E;

    // Requested torque (signed 12-bit value + always 0x0 in low nibble)
    /*static int16_t last_logged_final_torque_request = 0;
    if(final_torque_request != last_logged_final_torque_request){
      last_logged_final_torque_request = final_torque_request;
      //log_print_timestamp();
      serial->print(F("Sending torque request "));
      serial->print(final_torque_request);
      serial->print(F(" (speed: "));
      serial->print(inverter_status.speed / INVERTER_BITS_PER_RPM);
      serial->print(F(" rpm)"));
      serial->print(inverter_status.voltage / INVERTER_BITS_PER_VOLT);
      serial->print(F(" Volts)"));
      serial->println();
    }*/

    //Check if within bounds of 12 bit number   
//    if(final_torque_request >= -2048 && final_torque_request <= 2047){
//      outFrame.data.bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) | // Adds 1 to the beginning if negative
//          ((final_torque_request >> 4) // Equivalent to dividing by 2^4 (16), shifts bits left by 4
//          & 0x7f); // 
//      outFrame.data.bytes[3] = (final_torque_request << 4) & 0xf0;
//    } else {
      outFrame.data.bytes[2] = 0x00;
      outFrame.data.bytes[3] = 0x04;
//    }

    // MSB nibble: Runs through the sequence 0, 4, 8, C
    // LSB nibble: Precharge report (precedes actual precharge
    //             control)
    //   0: Discharging (5%)
    //   2: Precharge not started (1.4%)
    //   3: Precharging (0.4%)
    //   5: Starting discharge (3x10ms) (2.0%)
    //   7: Precharged (93%)
    outFrame.data.bytes[4] = 0x07 | (counter_1d4 << 6); // Bitshift counter left by 6 (11 -> 11000000, 10 -> 10000000)
    // First part is LSB nibble, counter controls MSB nibble
    
    //outFrame.data.bytes[4] = 0x02 | (counter_1d4 << 6);

    counter_1d4++;
    if(counter_1d4 >= 4)
      counter_1d4 = 0;

    //counter_1d4 is only ever 0, 1, 2, 3 and cycles for every loop (only ever takes up two bits)

    // MSB nibble:
    //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
    //      after the car has been shut off (6% total)
    //   4: Otherwise (94%)
    // LSB nibble:
    //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
    //      D (0.3%)
    //   2: Reverse gear related (13%)
    //   4: Forward gear related (21%)
    //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
    //outFrame.data.bytes[5] = 0x44;
    //outFrame.data.bytes[5] = 0x46;

    // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
    // 0x46 requires ~25 torque to start
    //outFrame.data.bytes[5] = 0x46;
    // 0x44 requires ~8 torque to start
    outFrame.data.bytes[5] = 0x46;

    // MSB nibble:
    //   In a drive cycle, this slowly changes between values (gen1):
    //     leaf_on_off.txt:
    //       5 7 3 2 0 1 3 7
    //     leaf_on_rev_off.txt:
    //       5 7 3 2 0 6
    //     leaf_on_Dx3.txt:
    //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
    //     leaf_on_stat_DRDRDR.txt:
    //       0 1 3 7
    //     leaf_on_Driveincircle_off.txt:
    //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7 
    //     leaf_on_wotind_off.txt:
    //       3 2 0 8 A B 3 7
    //     leaf_on_wotinr_off.txt:
    //       5 7 3 2 0 8 A B 3 7
    //     leaf_ac_charge.txt:
    //       4 6 E 6
    //   Possibly some kind of control flags, try to figure out
    //   using:
    //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq | ~/projects/leaf_tools/util/hex_to_ascii_binary.py
    //   2016:
    //     Has different values!
    // LSB nibble:
    //   0: Always (gen1)
    //   1:  (2016)

    // 2016 drive cycle:
    //   E0: to 0.15s
    //   E1: 2 messages
    //   61: to 2.06s (inverter is powered up and precharge
    //                 starts and completes during this)
    //   21: to 13.9s
    //   01: to 17.9s
    //   81: to 19.5s
    //   A1: to 26.8s
    //   21: to 31.0s
    //   01: to 33.9s
    //   81: to 48.8s
    //   A1: to 53.0s
    //   21: to 55.5s
    //   61: 2 messages
    //   60: to 55.9s
    //   E0: to end of capture (discharge starts during this)

    // This value has been chosen at the end of the hardest
    // acceleration in the wide-open-throttle pull, with full-ish
    // torque still being requested, in
    //   LeafLogs/leaf_on_wotind_off.txt
    //outFrame.data.bytes[6] = 0x00;

    // This value has been chosen for being seen most of the time
    // when, and before, applying throttle in the wide-open-throttle
    // pull, in
    //   LeafLogs/leaf_on_wotind_off.txt
    outFrame.data.bytes[6] = 0xE0;    //brake applied heavilly.

    // Value chosen from a 2016 log
    //outFrame.data.bytes[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
   // outFrame.data.bytes[6] = 0x01;
    //byte 6 brake signal
    uint16_t byte_7_1d4[] = {0x64, 0xA3, 0x6F, 0xA8};
    outFrame.data.bytes[7] = byte_7_1d4[counter_1d4];

    // Extra CRC
    nissan_crc(outFrame.data.bytes, 0x85);

    /*serial->print(F("Sending "));
    print_fancy_inFrame(inFrame);
    serial->println();*/

//    serial->print(outFrame.id, HEX);
//    serial->print(": ");
//    for(int i=0; i<8; i++){
//      serial->print(outFrame.data.bytes[i], HEX);
//      serial->print(" ");
//    }
//    serial->println();
    Can0.sendFrame(outFrame);
  }
}

//------------------------------------------------------------------------------------nissan_crc------------------------------------------------------------------------------------
// Cyclic redundancy check, used to ensure that message is transferred without packet loss
static void nissan_crc(uint8_t *data, uint8_t polynomial){
  // We want to process 8 bytes with the 8th byte being zero
  data[7] = 0;
  uint8_t crc = 0;
  for(int b=0; b<8; b++)
  {
    for(int i=7; i>=0; i--)
    {
      uint8_t bit = ((data[b] &(1 << i)) > 0) ? 1 : 0;
      if(crc >= 0x80) 
        crc = (byte)(((crc << 1) + bit) ^ polynomial);
      else 
        crc = (byte)((crc << 1) + bit);
    }
  }
  data[7] = crc;
}

/*static int8_t fahrenheit_to_celsius(uint16_t fahrenheit)
{
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if(result < -128)
    return -128;
  if(result > 127)
    return 127;
  return result;
}*/
