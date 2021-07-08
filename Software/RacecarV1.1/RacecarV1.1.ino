#include <due_can.h>  
#include <due_wire.h> 
#include <Wire_EEPROM.h> 
#include <Chrono.h>

#define MC  2      // Motor controller relay
#define Ign 3      // Ignition relay
#define Failsafe 4    // Failsafe relay
#define FailsafeChg 5   // Failsafe Charge Relay
#define SysMain1  6  // System Main 1 Relay (signals relay inside battery)
#define SysMain2  7  // System Main 2 Relay (signals relay inside battery)
#define PreChg  8    // Precharge Relay

int battery_voltage = 0;

struct InverterStatus {
  uint16_t voltage = 0;
  int16_t speed = 0;
  int8_t inverter_temperature = 0;
  int8_t motor_temperature = 0;
  bool error_state = false;
} inverter_status;

int inv_volts_local;
int inv_speed_local;
int16_t final_torque_request = 0;

CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames

#define INVERTER_BITS_PER_VOLT 2
#define INVERTER_BITS_PER_RPM 2

Chrono timer_Frames10 = Chrono();
Chrono timer_Frames100 = Chrono();
Chrono timer_hv = Chrono();
//Chrono timer_3sec = Chrono();

bool can_status = true;
bool Pch_Flag = false;
bool HV_Flag = false;
bool global_enable = true;
String readString;

const int accelPedalPin = A0;
const int minValueAccel = 255;
const int maxValueAccel = 700;
int accelerator = 0;

const int brakePedalPin = A1;
const int minValueBrake = 170;
const int maxValueBrake = 310;
int brake = 0;

void setup() {
  // put your setup code here, to run once:
  Can0.begin(CAN_BPS_500K);   // Inverter CAN
  Can0.watchFor();

  Can1.begin(CAN_BPS_500K);
  Can1.watchFor();

  Serial.begin(9600);
  
  pinMode(MC, OUTPUT);
  pinMode(Ign, OUTPUT);
  pinMode(Failsafe, OUTPUT);
  pinMode(FailsafeChg, OUTPUT);
  pinMode(SysMain1, OUTPUT);
  pinMode(SysMain2, OUTPUT);
  pinMode(PreChg, OUTPUT);

  digitalWrite(MC, HIGH);
  digitalWrite(Ign, HIGH);
  digitalWrite(Failsafe, HIGH);
  digitalWrite(FailsafeChg, HIGH);
  digitalWrite(SysMain1, HIGH);
  digitalWrite(SysMain2, HIGH);
  digitalWrite(PreChg, HIGH);

  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (timer_hv.hasPassed(1000)) {
    timer_hv.restart();
    HV_Con();
  } //control hv system

  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) {
   // Serial.println(readString);  //so you can see the captured string
    final_torque_request = readString.toInt();  //convert readString into a number

  }

  readString=""; //empty for next input
  
  CheckCAN();
  Msgs100ms();  //fire the 100ms can messages
  Msgs10ms();   //fire the 10ms can messages
}

void HV_Con()
{

  Serial.print("Inverter Voltage: ");
  Serial.println(inv_volts_local);
  Serial.print("Motor Speed: ");
  Serial.println(inv_speed_local);
  
  inv_volts_local=(inverter_status.voltage / INVERTER_BITS_PER_VOLT);
  inv_speed_local = (inverter_status.voltage / INVERTER_BITS_PER_RPM);
  
  if (global_enable && !Pch_Flag)  //if terminal 15 is on and precharge not enabled
  {
    Serial.println("MC/IGN/FS/Main2 RELAY ON");
    digitalWrite(MC, LOW);  //inverter power on
    digitalWrite(Ign, LOW);  //ignition relay on
    digitalWrite(Failsafe, LOW);
    digitalWrite(SysMain2, LOW);  //main contactor on (for 1 rail)
    digitalWrite(SysMain1, HIGH);
    if(inv_volts_local<200)
    {
      Serial.println("Precharge ON");
      digitalWrite(PreChg, LOW);  //precharge on
      Pch_Flag=true;
    }
  }
  if (global_enable && !HV_Flag && Pch_Flag)  //using inverter measured hv for initial tests. Will use ISA derived voltage in final version.
  {

    Serial.println("Precharging");
    if (inv_volts_local>340)
    {
      Serial.println("Main1 RELAY ON, Pchg OFF");
      digitalWrite(SysMain1, LOW);  //main contactor on
      digitalWrite(PreChg, HIGH);
      HV_Flag=true;  //hv on flag
    }
  }
  
  if (!global_enable)
  {
    Serial.println("Turn off everything");
    digitalWrite(PreChg, HIGH);  //precharge off
    digitalWrite(SysMain1, HIGH);  //main contactor off
    digitalWrite(SysMain2, HIGH);  //main contactor off
    digitalWrite(MC, HIGH);  //inverter power off
    digitalWrite(Ign, HIGH);  //ignition relay on
    digitalWrite(Failsafe, HIGH);
  }

}


void CheckCAN()
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//read incoming data from inverter//////////////////
//////////////////////////////////////////////////////
  if(Can0.available())
  {
    Can0.read(inFrame);
    //Serial.print("CAN ID");
    //Serial.println(inFrame.id, HEX);

    if(inFrame.id == 0x1db && inFrame.length == 8){
      //Serial.print("Battery Voltage?: ");
      //Serial.println(inFrame.data.bytes[2], HEX);
      //Serial.println(inFrame.data.bytes[3], HEX);
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
  
    if(inFrame.id == 0x55a && inFrame.length == 8){
     // last_received_from_inverter_timestamp = millis();
  
      inverter_status.inverter_temperature = fahrenheit_to_celsius(inFrame.data.bytes[2]);
      inverter_status.motor_temperature = fahrenheit_to_celsius(inFrame.data.bytes[1]);
      //Serial.println(inverter_status.inverter_temperature);
    }

  }

////////////////////////////////////////////////////////////////////////////////////////////////  
}

void Msgs10ms()                       //10ms messages here
{
  if(timer_Frames10.hasPassed(10))
  {
    //Serial.println("Sending msgs");
    timer_Frames10.restart();
    static uint8_t counter_11a_d6 = 0;
    static uint8_t counter_1d4 = 0;
    static uint8_t counter_1db = 0;
    
    outFrame.id = 0x11a;            // Set our transmission address ID
    outFrame.length = 8;            // Data payload 3 bytes
    outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outFrame.rtr=1;                 //No request
  
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
  
    /*Serial.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    Serial.println();*/
  
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
    static int16_t last_logged_final_torque_request = 0;
    if(final_torque_request != last_logged_final_torque_request){
      last_logged_final_torque_request = final_torque_request;
      //log_print_timestamp();
      Serial.print(F("Sending torque request "));
      Serial.print(final_torque_request);
      Serial.print(F(" (speed: "));
      Serial.print(inverter_status.speed / INVERTER_BITS_PER_RPM);
      Serial.print(F(" rpm)"));
      Serial.print(inverter_status.voltage / INVERTER_BITS_PER_VOLT);
      Serial.print(F(" Volts)"));
      Serial.println();
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

    /*Serial.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    Serial.println();*/

    Can0.sendFrame(outFrame);

    //We need to send 0x1db here with voltage measured by inverter

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

  }
}

void Msgs100ms(){
  if(timer_Frames100.hasPassed(100))
  {
    timer_Frames100.restart();
    if(can_status){

      //  digitalWrite(led, !digitalRead(led)); //toggle led everytime we fire the 100ms messages.
      
      outFrame.id = 0x50b;            // Set our transmission address ID
      outFrame.length = 7;            // Data payload 8 bytes
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
      outFrame.data.bytes[3]=0xc0;
      outFrame.data.bytes[4]=0x00;
      outFrame.data.bytes[5]=0x00;
      outFrame.data.bytes[6]=0x00;
    
      /*CONSOLE.print(F("Sending "));
      print_fancy_inFrame(inFrame);
      CONSOLE.println();*/
      Can0.sendFrame(outFrame); 
    }
}      
  
  
}

static int8_t fahrenheit_to_celsius(uint16_t fahrenheit)
{
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if(result < -128)
    return -128;
  if(result > 127)
    return 127;
  return result;
}

// Cyclic redundancy check, used to ensure that message is transferred without packet loss
static void nissan_crc(uint8_t *data, uint8_t polynomial)
{
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
