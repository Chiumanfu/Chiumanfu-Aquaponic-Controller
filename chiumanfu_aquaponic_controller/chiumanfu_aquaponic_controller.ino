/*
 *  A System Controller for Aquaponics, Hydroponics, Aquaculture
 *  Created by Chiu-Yuan Fang 2014

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  Use Arduino IDE v1.0.6. You will have problems if you try to compile with v1.6.x
 *  For use with an Arduino Mega 2560 rev3
 *  Ethernet module must be W5100 compatible
 *  The code is written for the Atlas pH stamp ver4.0 and DO stamp rev6.0. The newer EZO stamps use different syntax.
 *  If the RTC battery is changed, you may have to reset the time using the SetTime example sketch in the DS1307RTC folder.
 *  Build log is here : http://www.backyardaquaponics.com/forum/viewtopic.php?f=50&t=22607
 *
 *  Hookup schematic is here
 *  https://dl.dropboxusercontent.com/u/66807479/aquaponics/chiumanfu_aquaponic_controller_schematic.png
 *
 *  Changelog
 *  Chiumanfu Aquaponics Controller Rev1.1
 *    -Sets EEPROM defaults for a fresh arduino. Prevents hang up due to garbage value in EEPROM.
 *    -Added check for failed DS18B20 sensors.
 *    -Changed pinout for better PCB routing.
 *    -Minor rearrangement of LCD.
 *    -Added schematics
 *  Chiumanfu Aquaponics Controller Rev1.0
 *    -This is not really rev1 but is where I decided to start revision control.
 *    -Pinout altered for easier PCB layout.
 *    -Added check to make sure grow light and heater don't come on at the same time and blow the 10A fuse.
 *    -Comments added
*/

// Include Libraries
#include <LiquidCrystalFast.h> // For 40x4 LCD Display
#include <dht.h>               // For DHT-22 Temperature and Humidity Sensor
#include <OneWire.h>           // For DS18B20 Temperature Sensor
#include <DallasTemperature.h> // For DS18B20 Temperature Sensor
#include <NewPing.h>           // For HC-SR04 Ultrasonic Ranger
#include <Wire.h>              // For I2C Devices
#include <DS1307RTC.h>         // For DS1307 Real Time Clock
#include <Time.h>              // For Time functions
#include <SPI.h>               // For Ethernet and SD Card
#include <Ethernet.h>          // For Ethernet
#include <EthernetUdp.h>       // For UDP NTS sync
#include <Keypad.h>            // For Keypad
#include <EEPROMex.h>          // For EEPROM

// Declare Constants
const int ldrPin = A0;
const int dht6Pin = 38;
const int dht7Pin = 40;
const int temperaturePin = 42;
const int sumpPin = 48;
const int powerPin = 49;
const int flowPin = 47;
const int sonarPin = 44;
const int feederLEDPin = 46;
const int relay_lightPin = 37;
const int relay_pumpPin = 36;
const int relay_fanPin = 35;
const int relay_heaterPin = 34;
const int relay_feederPin = 33;
const int relay_6Pin = 32;
const int relay_7Pin = 31;
const int relay_8Pin = 30;

// Set Parameter Values
int flowSamplePeriod; // The sample period in milliseconds
int sonarDistanceMAX = 100; // Maximum sensor distance is rated at 400-500cm
int tankLevelMAX; // Maximum tank level to turn off pump
int relay_light_ldrMIN; // Minimum light level before the lights will turn on
int relay_light_ldrMAX; // Maximum light level before the lights will turn off
int relay_light_morningMIN; // Time in morning that lights will turn on
int relay_light_nightMAX; // Time in evening when lights will turn off
int relay_light_tempMIN; // Tank temp must be below for lights to turn on
int relay_light_tempMAX; // Tank temp must be above for lights to turn off. Hysteresis
int relay_pump_morningMIN; // Time in morning when pump cycle switches
int relay_pump_nightMAX; // Time in evening when pump cycle switches
int relay_pump_tempMIN; // Inside temp must be below for pump to turn off at night. Prevents growbeds acting as heatsink.
int relay_pump_tempMAX; // Inside temp must be above for pump to turn on at night. Hysteresis.
int relay_fan_tempMIN; // Air temp inside must be below to turn off fans. Hysteresis
int relay_fan_tempMAX; // Air temp inside must be above to turn on fans
int relay_heater_tempMIN; // Air temp inside must be below to turn on heater
int relay_heater_tempMAX; // Air temp inside must be above to turn off heater. Hysteresis
int keypadDebounce; // Keypad debounce. Default is 10mS
int timeZone; // Keypad debounce. Default is 10mS
unsigned long vibDelay; // Trigger siphon alert after no activity in 1 hour
unsigned long interruptTimeout; // Disable interrupt for timeout period while siphon dumps. 600000mS = 10 mins
unsigned long relay_feederDuration; // Duration that feeder motor is on after triggered
unsigned long relay_feederTimeout; // Duration that the feeder is inactive after a trigger. 60000mS = 1 mins

// Declare Variables
byte readDHT;
float t1Value;
float t2Value;
float t3Value;
float t4Value;
float t6Value;
float h6Value;
float t7Value;
float h7Value;
unsigned int ldrValue;
unsigned int flowValue;
unsigned int sonarValue;
float phValue;
float doValue;
char stamp_data_ph[30]; // char array for serial data from ph stamp
char stamp_data_do[30]; // char array for serial data from do stamp
byte holding; // counter for serial chars
byte i; //for loop counter
float ammoniaValue = 0;
float ammoniaValueOld = 0;
float nitriteValue = 0;
float nitriteValueOld = 0;
float nitrateValue = 0;
float nitrateValueOld = 0;
byte sumpValue = 1;
byte sumpValueOld = 1;
byte powerValue = 1;
byte powerValueOld = 1;
byte relay_lightValue = 0;
byte relay_lightValueOld = 0;
byte relay_pumpValue = 1;
byte relay_pumpValueOld = 1;
byte relay_fanValue = 1;
byte relay_fanValueOld = 1;
byte relay_heaterValue = 0;
byte relay_heaterValueOld = 0;
volatile byte relay_feederTrigger = 0;
unsigned long relay_feederTimer = 0;
unsigned int relay_feederValue = 0;
volatile unsigned long vib1Timer = 0;
byte vib1Value;
volatile unsigned long vib2Timer = 0;
byte vib2Value;
volatile unsigned long vib3Timer = 0;
byte vib3Value;
byte vibValue = 1;
byte vibValueOld = 1;
unsigned long calTimer = 0;
long calRemaining = 0;
int tankLevelCounter = 0; // Maximum tank level to turn off pump
int currentDay = 0;
unsigned long rtcEpoch;
char key; // Keypad input character
int keyInput = 0; // Keypad char string converted to int
float keyInputFloat = 00.00; // Keypad char string converted to float
float keyInputFloatTemp = 00.00; // Keypad char string converted to float
unsigned long keyInputLong = 0; // Keypad char string converted to long
int first_run = 0;

// Sensor setup
dht DHT; // Init DHT sensors
NewPing sonar(sonarPin, sonarPin, sonarDistanceMAX); // Init Sonar
OneWire oneWire(temperaturePin); // Init DS18B20 One Wire
DallasTemperature sensors(&oneWire); // Init Dallas Temp library

// ***** The DS18B20 device addresses need to be changed to match the DS18B20 sensors that you have
// ***** Every one has a unique ID. In the Arduino IDE, go to File/Examples/DallsTemperature/Tester.ino
// ***** Load the sketch with a single probe attached and record the deviceID.
DeviceAddress t1 = { 0x28, 0x92, 0x56, 0xD2, 0x05, 0x00, 0x00, 0xFB };
DeviceAddress t2 = { 0x28, 0x6E, 0x83, 0xD2, 0x05, 0x00, 0x00, 0x78 };
DeviceAddress t3 = { 0x28, 0x82, 0x25, 0xD3, 0x05, 0x00, 0x00, 0x0C };
DeviceAddress t4 = { 0x28, 0x50, 0x07, 0xD3, 0x05, 0x00, 0x00, 0xDD };
// DeviceAddress t5 = { 0x28, 0x2A, 0xC1, 0xD2, 0x05, 0x00, 0x00, 0xBB };

// LCD setup
LiquidCrystalFast lcd(29, 28, 27, 26, 24, 25, 22, 23); // rs,rw,en1,en2,d4,d5,d6,d7 Init LCD

// Keypad setup
const byte ROWS = 4; // Keypad four rows
const byte COLS = 4; // Keypad four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {A8, A9, A10, A11}; // Row pinouts of the keypad
byte colPins[COLS] = {A12, A13, A14, A15}; // Column pinouts of the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS ); // Initialize keypad

// Grovestream setup
char gsDomain[] = "grovestreams.com"; // The GroveStreams domain.

// ***** This value needs to be set according to the API key given to you by Grovestreams.
// ***** Go to the Observation Studio. Admin/API_Keys/Feed_Put_API_Key and copy paste your key here.
String gsApiKey = "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx";

//This value needs to be set according to the name you gave during grovestreams setup.
String gsComponentID = "Arduino";

const unsigned long gsUpdateFrequency = 900000; // GroveStreams update frequency 15min
unsigned long gsLastSuccessfulUploadTime = 0; // Timer for interval data streams
unsigned long gsRandomLastSuccessfulUploadTime = 0; // Timer for random data streams
unsigned long gsConnectAttemptTime = 0;
boolean gsLastConnected = false;
int gsFailedCounter = 0;

// Ethernet Setup
// ***** Set this MAC address for a unique value on your network.***********
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0x60, 0xA4 }; // Unique MAC address

// IPAddress ip(192,168,1,100); // Static IP outside routers DHCP range
unsigned int localPort = 8888;      // local port to listen for UDP packets
IPAddress timeServer(216, 228, 192, 69); // time-a.timefreq.bldrdoc.gov NTP server
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets 
EthernetClient client; // Initialize the Etherenet Client
EthernetUDP Udp;

void setup()
{
  // Read settings from EEPROM
  flowSamplePeriod = EEPROM.readInt(0);
  sonarDistanceMAX = EEPROM.readInt(2);
  tankLevelMAX = EEPROM.readInt(4);
  relay_light_ldrMIN = EEPROM.readInt(6);
  relay_light_ldrMAX = EEPROM.readInt(8);
  relay_light_morningMIN = EEPROM.readInt(10);
  relay_light_nightMAX = EEPROM.readInt(12);
  relay_light_tempMIN = EEPROM.readInt(14);
  relay_light_tempMAX = EEPROM.readInt(16);
  relay_pump_morningMIN = EEPROM.readInt(18);
  relay_pump_nightMAX = EEPROM.readInt(20);
  relay_pump_tempMIN = EEPROM.readInt(22);
  relay_pump_tempMAX = EEPROM.readInt(24);
  relay_fan_tempMIN = EEPROM.readInt(26);
  relay_fan_tempMAX = EEPROM.readInt(28);
  relay_heater_tempMIN = EEPROM.readInt(30);
  relay_heater_tempMAX = EEPROM.readInt(32);
  keypadDebounce = EEPROM.readInt(34);
  timeZone = EEPROM.readInt(36);
  vibDelay = EEPROM.readLong(38);
  interruptTimeout = EEPROM.readLong(42);
  relay_feederDuration = EEPROM.readLong(46);
  relay_feederTimeout = EEPROM.readLong(50);
  first_run = EEPROM.readInt(54);

  Serial.begin(115200); //For debug
  Serial3.begin(38400); // Init serial for pH sensor
  Serial2.begin(38400); // Init serial for DO sensor
  lcd.begin(40, 4); //Config LCD
  sensors.begin(); // Init DS18B20 sensors
  sensors.setResolution(t1, 12); // Set DS18B20 resolution
  sensors.setResolution(t2, 12); // Set DS18B20 resolution
  sensors.setResolution(t3, 12); // Set DS18B20 resolution
  sensors.setResolution(t4, 12); // Set DS18B20 resolution
  pinMode(ldrPin, INPUT);  // Set pin mode for LDR
  pinMode(sumpPin, INPUT);  // Set pin mode for sump float switch
  pinMode(powerPin, INPUT);  //Set pin mode for power sense
  pinMode(feederLEDPin,OUTPUT);  //Set pin mode for feeder LED
  digitalWrite(feederLEDPin, HIGH);
  digitalWrite(relay_lightPin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_pumpPin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_fanPin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_heaterPin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_feederPin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_6Pin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_7Pin, HIGH); // Required so relay pin starts in a known state.
  digitalWrite(relay_8Pin, HIGH); // Required so relay pin starts in a known state.
  pinMode(relay_lightPin, OUTPUT);  //Set pin mode for relay lights
  pinMode(relay_pumpPin, OUTPUT);  //Set pin mode for relay pump
  pinMode(relay_fanPin, OUTPUT);  //Set pin mode for relay fans
  pinMode(relay_heaterPin, OUTPUT);  //Set pin mode for relay water heater
  pinMode(relay_feederPin, OUTPUT);  //Set pin mode for relay feeder
  pinMode(relay_6Pin, OUTPUT);  //Set pin mode for spare relay
  pinMode(relay_7Pin, OUTPUT);  //Set pin mode for spare relay
  pinMode(relay_8Pin, OUTPUT);  //Set pin mode for spare relay
  TCCR5A=0; // reset timer/counter control register A for Flow Meter
  attachInterrupt(0, feeder, FALLING); // Config interrupt for demand feeder
  attachInterrupt(1, vib1, FALLING); // Config interrupt for bell siphon vib sensor
  attachInterrupt(4, vib2, FALLING); // Config interrupt for bell siphon vib sensor
  attachInterrupt(5, vib3, FALLING); // Config interrupt for bell siphon vib sensor
  keypad.setDebounceTime(keypadDebounce); // Set keypad debounce
  startEthernet(); //Start or Restart the Ethernet Connection
  Serial3.print("E"); // Set the pH board to single reading mode
  Serial3.print("\r"); // Send carriage return to end command
  Serial2.print("E"); // Set the DO board to single reading mode
  Serial2.print("\r"); // Send carriage return to end command
}

void loop()
{
  // Check to see if arduino is fresh. Set defaults.
  if (first_run != 1234) {
    flowSamplePeriod = 500; // The sample period in milliseconds
    sonarDistanceMAX = 50; // Maximum sensor distance is rated at 400-500cm
    tankLevelMAX = 5; // Maximum tank level to turn off pump
    relay_light_ldrMIN = 200; // Minimum light level before the lights will turn on
    relay_light_ldrMAX = 700; // Maximum light level before the lights will turn off
    relay_light_morningMIN = 10; // Time in morning that lights will turn on
    relay_light_nightMAX = 18; // Time in evening when lights will turn off
    relay_light_tempMIN = 12; // Tank temp must be below for lights to turn on
    relay_light_tempMAX = 16; // Tank temp must be above for lights to turn off. Hysteresis
    relay_pump_morningMIN = 8; // Time in morning when pump cycle switches
    relay_pump_nightMAX = 22; // Time in evening when pump cycle switches
    relay_pump_tempMIN = 10; // Inside temp must be below for pump to turn off at night. Prevents growbeds acting as heatsink.
    relay_pump_tempMAX = 14; // Inside temp must be above for pump to turn on at night. Hysteresis.
    relay_fan_tempMIN = 25; // Air temp inside must be below to turn off fans. Hysteresis
    relay_fan_tempMAX = 27; // Air temp inside must be above to turn on fans
    relay_heater_tempMIN = 10; // Tank temp inside must be below to turn on heater
    relay_heater_tempMAX = 14; // Tank temp inside must be above to turn off heater. Hysteresis
    keypadDebounce = 10; // Keypad debounce. Default is 10mS
    timeZone = -8;
    vibDelay = 3600000; // Trigger siphon alert after no activity in 1 hour
    interruptTimeout = 600000; // Disable interrupt for timeout period while siphon dumps. 600000mS = 10 mins
    relay_feederDuration = 1000; // Duration that feeder motor is on after triggered
    relay_feederTimeout = 60000; // Duration that the feeder is inactive after a trigger.
    first_run = 1234; // Flag that reset has happened
    EEPROM.updateInt(0,flowSamplePeriod);
    EEPROM.updateInt(2,sonarDistanceMAX);
    EEPROM.updateInt(4,tankLevelMAX);
    EEPROM.updateInt(6,relay_light_ldrMIN);
    EEPROM.updateInt(8,relay_light_ldrMAX);
    EEPROM.updateInt(10,relay_light_morningMIN);
    EEPROM.updateInt(12,relay_light_nightMAX);
    EEPROM.updateInt(14,relay_light_tempMIN);
    EEPROM.updateInt(16,relay_light_tempMAX);
    EEPROM.updateInt(18,relay_pump_morningMIN);
    EEPROM.updateInt(20,relay_pump_nightMAX);
    EEPROM.updateInt(22,relay_pump_tempMIN);
    EEPROM.updateInt(24,relay_pump_tempMAX);
    EEPROM.updateInt(26,relay_fan_tempMIN);
    EEPROM.updateInt(28,relay_fan_tempMAX);
    EEPROM.updateInt(30,relay_heater_tempMIN);
    EEPROM.updateInt(32,relay_heater_tempMAX);
    EEPROM.updateInt(34,keypadDebounce);
    EEPROM.updateInt(36,timeZone);
    EEPROM.updateLong(38,vibDelay);
    EEPROM.updateLong(42,interruptTimeout);
    EEPROM.updateLong(46,relay_feederDuration);
    EEPROM.updateLong(50,relay_feederTimeout);
    EEPROM.updateInt(54,first_run);
  }
  
  // DS18B20 Sensors
  sensors.requestTemperatures(); // read temperatures from DS18B20 sensors
  t1Value = sensors.getTempC(t1);
  t2Value = sensors.getTempC(t2);
  t3Value = sensors.getTempC(t3);
  t4Value = sensors.getTempC(t4);
  //t5Value = sensors.getTempC(t5);
  lcd.setCursor(0, 0);
  lcd.print(F("T1:     "));
  lcd.setCursor(3, 0);
  if (t1Value < -100) {
    lcd.print(F("ERR"));
  } else {
    lcd.print(t1Value, 1);
  }
  
  lcd.setCursor(0, 1);
  lcd.print(F("T2:     "));
  lcd.setCursor(3, 1);
  if (t2Value < -100) {
    lcd.print(F("ERR"));
  } else {
    lcd.print(t2Value, 1);
  }
  
  lcd.setCursor(0, 2);
  lcd.print(F("T3:     "));
  lcd.setCursor(3, 2);
  if (t3Value < -100) {
    lcd.print(F("ERR"));
  } else {
    lcd.print(t3Value, 1);
  }
  
  lcd.setCursor(0, 3);
  lcd.print(F("T4:     "));
  lcd.setCursor(3, 3);
  if (t4Value < -100) {
    lcd.print(F("ERR"));
  } else {  
    lcd.print(t4Value, 1);
  }
  
  // DHT Sensor 6
  readDHT = DHT.read22(dht6Pin); // read temps and humidity from inside DHT-22
  switch (readDHT){
    case DHTLIB_OK:
      t6Value = DHT.temperature;
      h6Value = DHT.humidity;
      lcd.setCursor(8, 2);
      lcd.print("IN:      H:  % ");
      lcd.setCursor(11, 2);
      lcd.print(t6Value, 1);
      lcd.setCursor(19, 2);
      lcd.print(h6Value, 0);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      lcd.setCursor(8, 2);
      lcd.print("T6:ERROR    ");
      break;
    case DHTLIB_ERROR_TIMEOUT: 
      lcd.setCursor(8, 2);
      lcd.print("T6:ERROR    ");
      break;
    default:
      break;
  }

  // DHT Sensor 7
  readDHT = DHT.read22(dht7Pin); // read temps and humidity from outside DHT-22
  switch (readDHT){
    case DHTLIB_OK:
      t7Value = DHT.temperature;
      h7Value = DHT.humidity;
      lcd.setCursor(8, 3);
      lcd.print("OT:      H:  % ");
      lcd.setCursor(11, 3);
      lcd.print(t7Value, 1);
      lcd.setCursor(19, 3);
      lcd.print(h7Value, 0);
      break;
    case DHTLIB_ERROR_CHECKSUM: 
      lcd.setCursor(8, 3);
      lcd.print("T7:ERROR    ");
      break;
    case DHTLIB_ERROR_TIMEOUT: 
      lcd.setCursor(8, 3);
      lcd.print("T7:ERROR    ");
      break;
    default: 
      break;
  }

  // pH Sensor board rev4.0. Newer EZO boards will not work with this code.
  Serial3.print(t1Value); // Send water temp for calibrated pH reading
  Serial3.print("\r"); // Send carriage return to end command
  delay(700); // Wait for serial data
  holding = 0; // Reset buffer counter
  if(Serial3.available() > 1) { // If there is serial data in buffer
    holding = Serial3.available(); // Set buffer counter to number of bytes
    for(i=0; i<holding; i++){ // For loop to read out number of bytes in buffer
      stamp_data_ph[i]= Serial3.read(); // Read serial data to char array
    }
  } else {
    Serial.println(F("pH ERR")); // Error if no data rx in serial buffer
  }
  phValue = atof(stamp_data_ph); // Convert string to float
  lcd.setCursor(8, 0);
  lcd.print("pH:      ");
  lcd.setCursor(11, 0);
  lcd.print(phValue);

  // DO Sensor board rev6.0. Newer EZO boards will not work with this code.
  Serial2.print(t1Value); // Send water temp for calibrated DO reading
  Serial2.print("\r"); // Send carriage return to end command
  Serial2.print("R"); 
  Serial2.print("\r"); 
  delay(700); // Wait for serial data
  holding = 0; // Reset buffer counter
  if(Serial2.available() > 1) { // If there is serial data in buffer
    holding = Serial2.available(); // Set buffer counter to number of bytes
    for(i=0; i<holding; i++){ // For loop to read out number of bytes in buffer
      stamp_data_do[i]= Serial2.read(); // Read serial data to char array
    }
  } else {
    Serial.println(F("DO ERR")); // Error if no data rx in serial buffer
  }
  doValue = atof(stamp_data_do); // Convert string to float
  lcd.setCursor(8, 1);
  lcd.print("DO:      ");
  lcd.setCursor(11, 1);
  lcd.print(doValue);

  // LDR Sensor
  ldrValue = analogRead(ldrPin); // Read analog value
  lcd.setCursor(17, 1);
  lcd.print("L:    ");
  lcd.setCursor(19, 1);
  lcd.print(ldrValue);

  // Flow Sensor
  flowValue = 0; // Reset the variable
  bitSet(TCCR5B ,CS12); // Counter Clock source is external pin
  bitSet(TCCR5B ,CS11); // Clock on rising edge
  bitSet(TCCR5B ,CS10); // Clock on rising edge
  TCNT5=0; // Reset counter
  delay(flowSamplePeriod); // Count for sample period
  TCCR5B = 0; // Stop the counting
  flowValue = TCNT5; // Copy the variable
  lcd.setCursor(23, 0);
  lcd.print("FLO:     ");
  lcd.setCursor(27, 0);
  lcd.print(flowValue);

  // Sonar Sensor
  sonarValue = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  sonarValue = sonarValue / US_ROUNDTRIP_CM; // Convert to distance
  if(sonarValue == 0){ // If the returned value is too far, set the variable for max distance instead of 0.
    sonarValue = sonarDistanceMAX;
  }
  if(sonarValue <= tankLevelMAX) {
    tankLevelCounter++;
  } else {
    tankLevelCounter = 0;
  }
  lcd.setCursor(17, 0);
  lcd.print("D:    ");
  lcd.setCursor(19, 0);
  lcd.print(sonarValue);

  // Real Time Clock
  tmElements_t tm; // Calls RTC lib
  if (RTC.read(tm)) { // Read DS1307

  print2digits(tm.Hour);
  Serial.write(':');
  print2digits(tm.Minute);
  Serial.write(':');
  print2digits(tm.Second);
  Serial.print(" - ");
  Serial.print(tm.Day);
  Serial.write('/');
  Serial.print(tm.Month);
  Serial.write('/');
  Serial.print(tmYearToCalendar(tm.Year));
  Serial.print(",");

  } else {
    if (RTC.chipPresent()) {
    Serial.println(F("DS1307 Stopped"));
    } else {
    Serial.println(F("DS1307 read error!"));
    }
  }

  lcd.setCursor(32, 0);
  lcd2digits(tm.Hour);
  lcd.print(':');
  lcd2digits(tm.Minute);
  lcd.print(':');
  lcd2digits(tm.Second);
  
  if(tm.Day != currentDay) { // Check for midnight
    syncNTP();               // Call Network Time Protocol sync
  }
  currentDay = tm.Day;       // Reset flag

  // Power Sensor
  powerValue = digitalRead(powerPin); // read the power sense pin to detect mains power failure.
  lcd.setCursor(23, 1);
  lcd.print("POWR:  ");
  lcd.setCursor(28, 1);
  lcd.print(powerValue);

  // Relay Lights
  if (powerValue == HIGH && ldrValue <= relay_light_ldrMIN && tm.Hour >= relay_light_morningMIN && tm.Hour <= relay_light_nightMAX && t1Value <= relay_light_tempMIN){
    digitalWrite(relay_lightPin, LOW); // turn lights on if power is good and it's daytime and the tank temp is cool.
    relay_lightValue = 1;
  } else if (powerValue == LOW || ldrValue >= relay_light_ldrMAX || tm.Hour < relay_light_morningMIN || tm.Hour > relay_light_nightMAX || t1Value >= relay_light_tempMAX){
    digitalWrite(relay_lightPin, HIGH); // turn lights off if power is lost or it's night time or the tank temp is too hot.
    relay_lightValue = 0;
  }
  lcd.setCursor(30, 2);
  lcd.print("LIT:  ");
  lcd.setCursor(34, 2);
  lcd.print(relay_lightValue);

  // Relay Pump
  sumpValue = digitalRead(sumpPin);
  if (sumpValue == 0 || tankLevelCounter >= 5){
    digitalWrite(relay_pumpPin, LOW); // turn water pump off if the water level is too high for 5 continuous loops or sump is empty.
    relay_pumpValue = 0;
  } else if (t6Value <= relay_pump_tempMIN && tm.Hour <= relay_pump_morningMIN && tm.Hour >= relay_pump_nightMAX){
    digitalWrite(relay_pumpPin, LOW); // turn water pump off if the inside temp is cold and it's night time.
    relay_pumpValue = 0;
  } else if (relay_pumpValue == 0 && t6Value <= relay_pump_tempMAX && tm.Hour <= relay_pump_morningMIN && tm.Hour >= relay_pump_nightMAX){
    digitalWrite(relay_pumpPin, LOW); // hysteresis so the pump does not toggle if temperatures fluctuate.
    relay_pumpValue = 0;
  } else {
    digitalWrite(relay_pumpPin, HIGH); // all other conditions pump is on. Pump on normally closed contacts.
    relay_pumpValue = 1;
  }
  lcd.setCursor(23, 2);
  lcd.print(F("PUMP:  "));
  lcd.setCursor(28, 2);
  lcd.print(relay_pumpValue);
  lcd.setCursor(23, 3);
  lcd.print(F("SUMP:  "));
  lcd.setCursor(28, 3);
  lcd.print(sumpValue);
  
  // Relay Fan
  if(powerValue == HIGH && t6Value >= relay_fan_tempMAX){
    digitalWrite(relay_fanPin, LOW); // turn fan on if inside temp is high.
    relay_fanValue = 1;
  } else if (powerValue == LOW || t6Value <= relay_fan_tempMIN){
    digitalWrite(relay_fanPin, HIGH); // turn fan off if inside temp is cold.
    relay_fanValue = 0;
  }
  lcd.setCursor(30, 1);
  lcd.print("FAN:  ");
  lcd.setCursor(34, 1);       
  lcd.print(relay_fanValue);
  
  // Relay Heater
  if (powerValue == HIGH && t1Value <= relay_heater_tempMIN && relay_lightValue == 0){
    digitalWrite(relay_heaterPin, LOW); // turn water heater on if tank temperature is too low
    relay_heaterValue = 1;
  } else if (powerValue == LOW || t1Value >= relay_heater_tempMAX || relay_lightValue == 1){
    digitalWrite(relay_heaterPin, HIGH); // turn water heater off if tank temperature is high
    relay_heaterValue = 0;
  }
  lcd.setCursor(30, 3);
  lcd.print("HTR:  ");
  lcd.setCursor(34, 3);       
  lcd.print(relay_heaterValue);
  
  // Relay Feeder
  if (relay_feederTrigger == 1){ // if feeder switch was hit, start feed motor
    relay_feederValue++;
    digitalWrite(feederLEDPin, LOW);
    digitalWrite(relay_feederPin, LOW);
    delay(relay_feederDuration); // this code is blocking to ensure accurate timing
    digitalWrite(relay_feederPin, HIGH);
    relay_feederTimer = millis();
    relay_feederTrigger = 0;
  } else {
    digitalWrite(relay_feederPin, HIGH);
  }

  if ((millis() - relay_feederTimer) >= relay_feederTimeout){ // wait timeout to reset feeder
    EIFR = (1 << INTF4); // clear pending interupts in queue
    attachInterrupt(0, feeder, FALLING); // enable interrupt
    digitalWrite(feederLEDPin, HIGH);
  }

  // Spare Relays
  digitalWrite(relay_6Pin, HIGH);
  digitalWrite(relay_7Pin, HIGH);
  digitalWrite(relay_8Pin, HIGH);

  // vib Sensor 1
  if ((millis() - vib1Timer) >= vibDelay){ // trigger if no action for 1 hour
    vib1Value = 0;
  } else {
    vib1Value = 1;
  }
  if ((millis() - vib1Timer) >= interruptTimeout){ // wait 10 minutes to allow time for siphon to finished dumping
    EIFR = (1 << INTF5); // clear pending interupts in queue
    attachInterrupt(1, vib1, FALLING); // enable interrupt
  }
  lcd.setCursor(36, 1);
  lcd.print("V1: ");
  lcd.setCursor(39, 1);       
  lcd.print(vib1Value);

  // vib Sensor 2
  if ((millis() - vib2Timer) >= vibDelay){ // trigger if no action for 1 hour
    vib2Value = 0;
  } else {
    vib2Value = 1;
  }
  if ((millis() - vib2Timer) >= interruptTimeout){ // wait 10 minutes to allow time for siphon to finished dumping
    EIFR = (1 << INTF2); // clear pending interupts in queue
    attachInterrupt(4, vib2, FALLING); // enable interrupt
  }
  lcd.setCursor(36, 2);
  lcd.print("V2: ");
  lcd.setCursor(39, 2);       
  lcd.print(vib2Value);

  // vib Sensor 3
  if ((millis() - vib3Timer) >= vibDelay){ // trigger if no action for 1 hour
    vib3Value = 0;
  } else {
    vib3Value = 1;
  }
  if ((millis() - vib3Timer) >= interruptTimeout){ // wait 10 minutes to allow time for siphon to finished dumping
    EIFR = (1 << INTF3); // clear pending interupts in queue
    attachInterrupt(5, vib3, FALLING); // enable interrupt
  }
  lcd.setCursor(36, 3);
  lcd.print("V3: ");
  lcd.setCursor(39, 3);       
  lcd.print(vib3Value);

  if (vib1Value == 0 || vib2Value == 0 || vib3Value == 0){ // capture all three siphon status for reporting to grovestreams
    vibValue = 0;
  } else {
    vibValue = 1;
  }

   // Create strings from floats for Grovestream string
  char temp1[6] = {0}; //Initialize buffer to nulls
  dtostrf(t1Value, 6, 2, temp1); //Convert float to string
  String t1ValueS(temp1);
  t1ValueS.trim();

  char temp2[6] = {0}; //Initialize buffer to nulls
  dtostrf(t2Value, 6, 2, temp2); //Convert float to string
  String t2ValueS(temp2);
  t2ValueS.trim();

  char temp3[6] = {0}; //Initialize buffer to nulls
  dtostrf(t3Value, 6, 2, temp3); //Convert float to string
  String t3ValueS(temp3);
  t3ValueS.trim();
  
  char temp4[6] = {0}; //Initialize buffer to nulls
  dtostrf(t4Value, 6, 2, temp4); //Convert float to string
  String t4ValueS(temp4);
  t4ValueS.trim();
  
  char temp6[6] = {0}; //Initialize buffer to nulls
  dtostrf(t6Value, 6, 2, temp6); //Convert float to string
  String t6ValueS(temp6);
  t6ValueS.trim();

  char temp7[6] = {0}; //Initialize buffer to nulls
  dtostrf(t7Value, 6, 2, temp7); //Convert float to string
  String t7ValueS(temp7);
  t7ValueS.trim();
  
  char temp8[6] = {0}; //Initialize buffer to nulls
  dtostrf(h6Value, 6, 2, temp8); //Convert float to string
  String h6ValueS(temp8);
  h6ValueS.trim();

  char temp9[6] = {0}; //Initialize buffer to nulls
  dtostrf(h7Value, 6, 2, temp9); //Convert float to string
  String h7ValueS(temp9);
  h7ValueS.trim();

  char temp10[6] = {0}; //Initialize buffer to nulls
  dtostrf(phValue, 6, 2, temp10); //Convert float to string
  String phValueS(temp10);
  phValueS.trim();

  char temp11[6] = {0}; //Initialize buffer to nulls
  dtostrf(doValue, 6, 2, temp11); //Convert float to string
  String doValueS(temp11);
  doValueS.trim();

  char temp12[6] = {0}; //Initialize buffer to nulls
  dtostrf(ammoniaValue, 6, 2, temp12); //Convert float to string
  String ammoniaValueS(temp12);
  ammoniaValueS.trim();
  
  char temp13[6] = {0}; //Initialize buffer to nulls
  dtostrf( nitriteValue, 6, 2, temp13); //Convert float to string
  String nitriteValueS(temp13);
  nitriteValueS.trim();
  
  char temp14[6] = {0}; //Initialize buffer to nulls
  dtostrf(nitrateValue, 6, 2, temp14); //Convert float to string
  String nitrateValueS(temp14);
  nitrateValueS.trim();
  
  String ldrValueS(ldrValue); //Convert int to string
  String flowValueS(flowValue); //Convert int to string
  String sonarValueS(sonarValue); //Convert int to string
  String sumpValueS(sumpValue); //Convert int to string
  String powerValueS(powerValue); //Convert int to string
  String relay_lightValueS(relay_lightValue); //Convert int to string
  String relay_pumpValueS(relay_pumpValue); //Convert int to string
  String relay_fanValueS(relay_fanValue); //Convert int to string
  String relay_heaterValueS(relay_heaterValue); //Convert int to string
  String relay_feederValueS(relay_feederValue); //Convert int to string
  String vibValueS(vibValue); //Convert int to string

  // Send data to Grovestream
  while(client.available()) { // Send Ethernet status to serial monitor
    char c = client.read();
    Serial.print(c);
  }
  
  if(!client.connected() && gsLastConnected) { // Disconnect from GroveStreams
    Serial.println(F("...disconnected"));
    client.stop();
  }

  if(!client.connected() && (millis() - gsLastSuccessfulUploadTime > gsUpdateFrequency)) { // wait 15 minutes between PUT
    gsConnectAttemptTime = millis();
    if (client.connect(gsDomain, 80)) { // Connect to grovestream server
      String url = "PUT /api/feed?compId=" + gsComponentID;
      url += "&api_key=" + gsApiKey;
      url += "&t1=" + t1ValueS;
//      url += "&t2=" + t2ValueS;
      url += "&t3=" + t3ValueS;
//      url += "&t4=" + t4ValueS;
      url += "&t6=" + t6ValueS;
      url += "&t7=" + t7ValueS;
      url += "&h6=" + h6ValueS;
      url += "&h7=" + h7ValueS;
      url += "&ph=" + phValueS;
      url += "&do=" + doValueS;
      url += "&ldr=" + ldrValueS;
      url += "&flow=" + flowValueS;
      url += "&sonar=" + sonarValueS;
      url += "&feeder=" + relay_feederValueS;
      url += " HTTP/1.1";
      client.println(url);  //Send the url with temp readings in one println(..) to decrease the chance of dropped packets
      client.println("Host: " + String(gsDomain));
      client.println(F("Connection: close"));
//      client.print(F("X-Forwarded-For: ")); //Include these two lines if you have more than one device uploading behind
//      client.println(ip);                   // your outward facing router (avoids the GS 10 second upload rule)
      client.println(F("Content-Type: application/json"));
      client.println();
      
      Serial.println(url); // for debug
   
      if (client.connected()) {
        gsLastSuccessfulUploadTime = gsConnectAttemptTime;
        gsFailedCounter = 0;
        relay_feederValue = 0; // reset feeded counter
      } else {
      gsFailedCounter++; // Connection failed. Increase failed counter
      Serial.println("Connection to GroveStreams failed ("+String(gsFailedCounter, DEC)+")");  
      }
    } else {
    gsFailedCounter++; // Connection failed. Increase failed counter
    Serial.println("Connection to GroveStreams Failed ("+String(gsFailedCounter, DEC)+")");  
    }
  }

  //Send random data to Grovestreams
  if((sumpValue != sumpValueOld || powerValue != powerValueOld || relay_lightValue != relay_lightValueOld || relay_pumpValue != relay_pumpValueOld || relay_fanValue != relay_fanValueOld || relay_heaterValue != relay_heaterValueOld || vibValue != vibValueOld)
// || ammoniaValue != ammoniaValueOld || nitriteValue != nitriteValueOld || nitrateValue != nitrateValueOld
    && (millis() - gsLastSuccessfulUploadTime >= 10000)
      && ((millis() - gsLastSuccessfulUploadTime) <= (gsUpdateFrequency - 10000))
        && (millis() - gsRandomLastSuccessfulUploadTime >= 10000)) { // send random data PUT if values changed and its been 10 seconds from the last PUT and it is 10 seconds from the next PUT and a random data PUT hasn't been sent in the last 10 seconds.
    if(!client.connected()) {
      if(client.connect(gsDomain, 80)) {
        String url = "PUT /api/feed?compId=" + gsComponentID;
        url += "&api_key=" + gsApiKey;
        url += "&sump=" + sumpValueS;
        url += "&power=" + powerValueS;
        url += "&light=" + relay_lightValueS;
        url += "&pump=" + relay_pumpValueS;
        url += "&fan=" + relay_fanValueS;
        url += "&heater=" + relay_heaterValueS;
        url += "&vib=" + vibValueS;
 //       url += "&ammonia=" + ammoninaValueS;
 //       url += "&nitrite=" + nitriteValueS;
 //       url += "&nitrate=" + nitrateValueS;
        url += " HTTP/1.1";
        client.println(url);  //Send the url with temp readings in one println(..) to decrease the chance of dropped packets
        client.println("Host: " + String(gsDomain));
        client.println(F("Connection: close"));
        client.println(F("Content-Type: application/json"));
        client.println();

        Serial.println(url); // for debug
        
        sumpValueOld = sumpValue; // track change
        powerValueOld = powerValue; // track change
        relay_lightValueOld = relay_lightValue; // track change
        relay_pumpValueOld = relay_pumpValue; // track change
        relay_fanValueOld = relay_fanValue; // track change
        relay_heaterValueOld = relay_heaterValue; // track change
        vibValueOld = vibValue; // track change
        ammoniaValueOld = ammoniaValue; // track change
        nitriteValueOld = nitriteValue; // track change
        nitrateValueOld = nitrateValue; // track change

        if (client.connected()) {
          gsRandomLastSuccessfulUploadTime = millis();
          gsFailedCounter = 0;
        } else {
          gsFailedCounter++; // Connection failed. Increase failed counter
          Serial.println("Connection to GroveStreams failed ("+String(gsFailedCounter, DEC)+")");  
        }      
      } else {
        gsFailedCounter++; // Connection failed. Increase failed counter
        Serial.println("Connection to GroveStreams Failed ("+String(gsFailedCounter, DEC)+")");  
      }
    }
  }
  
  if (gsFailedCounter > 3 ) { // Check if Arduino Ethernet needs to be restarted
  startEthernet();
  }

  gsLastConnected = client.connected();

// debug
  Serial.print(F("Ram:"));
  Serial.print(freeRam());
  Serial.print(F(","));
  Serial.print(F("Millis:"));
  Serial.print(millis());
  Serial.println(F(","));
    
  // Keypad Input to enter menu
  key = keypad.getKey();
  if(key) {
    homeScreen();
  }
  
}


void homeScreen() {
  lcd.clear();
  lcd.setCursor(16, 0);
  lcd.print(F("Chiumanfu"));
  lcd.setCursor(16, 1);
  lcd.print(F("Aquaponic"));
  lcd.setCursor(17, 2);
  lcd.print(F("System"));
  lcd.setCursor(16, 3);
  lcd.print(F("Controller"));
  delay(3000);
  mainMenu();
}

void mainMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Press A to Enter Water Test Values"));
  lcd.setCursor(0, 1);
  lcd.print(F("Press B for Settings"));
  lcd.setCursor(0, 2);
  lcd.print(F("Press C for Calibration"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press D to Exit"));
  key = keypad.waitForKey();
  if(key == 'A') {
    testScreen();
  }
  else if(key == 'B') {
    settingScreen();
  }
  else if(key == 'C') {
    calibrationScreen();
  }
  else {
    loop();
  }
}

void testScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("1:Ammonia"));
  lcd.setCursor(0, 1);
  lcd.print(F("2:Nitrite"));
  lcd.setCursor(0, 2);
  lcd.print(F("3:Nitrate"));
  lcd.setCursor(0, 3);
  lcd.print(F("D:Main Menu"));
  key = keypad.waitForKey();
  switch(key) {
    
    case 'D':
      mainMenu();
      break;
      
    case '1':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Ammonia Measured Level [ppm]"));
      lcd.setCursor(0, 1);
      lcd.print(F("Current = "));
      lcd.print(ammoniaValue);
      lcd.setCursor(0, 2);
      lcd.print(F("Enter Value = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(13, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = keypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if(key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(13, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while(key >= '0' && key <= '9');
      if(key == 'A') {
        ammoniaValue = keyInputFloatTemp;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Ammonia Current Level = "));
        lcd.print(ammoniaValue);
        lcd.setCursor(0, 1);
        lcd.print(F("Data Recorded"));
        keyInputFloat = 0;
        delay(3000);
        testScreen();
      } else {
        keyInputFloat = 0;
        testScreen();
      }
      break;
      
    case '2':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Nitrite Measured Level [ppm]"));
      lcd.setCursor(0, 1);
      lcd.print(F("Current = "));
      lcd.print(nitriteValue);
      lcd.setCursor(0, 2);
      lcd.print(F("Enter Value = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(13, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = keypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if(key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(13, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while(key >= '0' && key <= '9');
      if(key == 'A') {
        nitriteValue = keyInputFloatTemp;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Nitrite Current Level = "));
        lcd.print(nitriteValue);
        lcd.setCursor(0, 1);
        lcd.print(F("Data Recorded"));
        keyInputFloat = 0;
        delay(3000);
        testScreen();
      } else {
        keyInputFloat = 0;
        testScreen();
      }
      break;
      
    case '3':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Nitrate Measured Level [ppm]"));
      lcd.setCursor(0, 1);
      lcd.print(F("Current = "));
      lcd.print(nitrateValue);
      lcd.setCursor(0, 2);
      lcd.print(F("Enter Value = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(13, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = keypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if(key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(13, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while(key >= '0' && key <= '9');
      if(key == 'A') {
        nitrateValue = keyInputFloatTemp;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Nitrate Current Level = "));
        lcd.print(nitrateValue);
        lcd.setCursor(0, 1);
        lcd.print(F("Data Recorded"));
        keyInputFloat = 0;
        delay(3000);
        testScreen();
      } else {
        keyInputFloat = 0;
        testScreen();
      }
      break;
  }
}

void settingScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("1:Tank"));
  lcd.setCursor(0, 1);
  lcd.print(F("2:Grow Light"));
  lcd.setCursor(0, 2);
  lcd.print(F("3:Water Pump"));
  lcd.setCursor(0, 3);
  lcd.print(F("4:Fans and Heater"));
  lcd.setCursor(20, 0);
  lcd.print(F("5:Siphons"));
  lcd.setCursor(20, 1);
  lcd.print(F("6:Feeder"));
  lcd.setCursor(20, 2);
  lcd.print(F("7:System"));
  lcd.setCursor(20, 3);
  lcd.print(F("D:Main Menu"));
  key = keypad.waitForKey();
  switch(key) {
    
    case 'D': //top
      mainMenu();
      break;
      
    case '1': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Tank Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:flowSamplePeriod"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:sonarDistanceMAX"));
      lcd.setCursor(0, 3);
      lcd.print(F("3:tankLevelMAX"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Time for the flow meter to sample [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(flowSamplePeriod);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 500"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            flowSamplePeriod = keyInput;
            EEPROM.updateInt(0,flowSamplePeriod);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("flowSamplePeriod = "));
            lcd.print(flowSamplePeriod);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Maximum sonar distance to measure [cm]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(sonarDistanceMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 200"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            sonarDistanceMAX = keyInput;
            EEPROM.updateInt(2,sonarDistanceMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("sonarDistanceMAX = "));
            lcd.print(sonarDistanceMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
            
        case '3': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max Water Level. Smaller is higher [cm]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(tankLevelMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 2"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            tankLevelMAX = keyInput;
            EEPROM.updateInt(4,tankLevelMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("tankLevelMAX = "));
            lcd.print(tankLevelMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
      }
      break;

    case '2': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Grow Light Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:ldrMIN"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:ldrMAX"));
      lcd.setCursor(0, 3);
      lcd.print(F("3:morningMIN"));
      lcd.setCursor(20, 0);
      lcd.print(F("4:nightMAX"));
      lcd.setCursor(20, 1);
      lcd.print(F("5:tempMIN"));
      lcd.setCursor(20, 2);
      lcd.print(F("6:tempMAX"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
        
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Min ambient light to turn on grow light"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_ldrMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 200"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_ldrMIN = keyInput;
            EEPROM.updateInt(6,relay_light_ldrMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_ldrMIN = "));
            lcd.print(relay_light_ldrMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
          case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max ambient light to turn off grow light"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_ldrMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 900"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_ldrMAX = keyInput;
            EEPROM.updateInt(8,relay_light_ldrMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_ldrMAX = "));
            lcd.print(relay_light_ldrMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
          case '3': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Grow light can turn on after this hour"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_morningMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 10"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_morningMIN = keyInput;
            EEPROM.updateInt(10,relay_light_morningMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_morningMIN = "));
            lcd.print(relay_light_morningMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
          case '4': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Grow light will turn off after this hour"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_nightMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 18"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_nightMAX = keyInput;
            EEPROM.updateInt(12,relay_light_nightMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_nightMAX = "));
            lcd.print(relay_light_nightMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
          case '5':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Min inside temp for light to turn on"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_tempMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 12"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_tempMIN = keyInput;
            EEPROM.updateInt(14,relay_light_tempMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_tempMIN = "));
            lcd.print(relay_light_tempMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
          case '6':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max inside temp for light to turn off"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_light_tempMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 16"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_light_tempMAX = keyInput;
            EEPROM.updateInt(16,relay_light_tempMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_light_tempMAX = "));
            lcd.print(relay_light_tempMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
      }
      break;
      
    case '3': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Pump Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:morningMIN"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:nightMAX"));
      lcd.setCursor(0, 3);
      lcd.print(F("3:tempMIN"));
      lcd.setCursor(20, 0);
      lcd.print(F("4:tempMAX"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Morning hour when the pump cycle changes"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_pump_morningMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 8"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_pump_morningMIN = keyInput;
            EEPROM.updateInt(18,relay_pump_morningMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_pump_morningMIN = "));
            lcd.print(relay_pump_morningMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Evening hour when the pump cycle changes"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_pump_nightMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 22"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_pump_nightMAX = keyInput;
            EEPROM.updateInt(20,relay_pump_nightMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_pump_nightMAX = "));
            lcd.print(relay_pump_nightMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
        case '3': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Min inside temp to turn off pump at nite"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_pump_tempMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 10"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_pump_tempMIN = keyInput;
            EEPROM.updateInt(22,relay_pump_tempMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_pump_tempMIN = "));
            lcd.print(relay_pump_tempMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
        case '4': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max inside temp to turn on pump at night"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_pump_tempMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 14"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_pump_tempMAX = keyInput;
            EEPROM.updateInt(24,relay_pump_tempMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_pump_tempMAX = "));
            lcd.print(relay_pump_tempMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
      }
      break;

    case '4': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Fan Heater Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:fan_tempMIN"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:fan_tempMAX"));
      lcd.setCursor(0, 3);
      lcd.print(F("3:heater_tempMIN"));
      lcd.setCursor(20, 0);
      lcd.print(F("4:heater_tempMAX"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Min ambient temp to turn off fan"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_fan_tempMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 20"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_fan_tempMIN = keyInput;
            EEPROM.updateInt(26,relay_fan_tempMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_fan_tempMIN = "));
            lcd.print(relay_fan_tempMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max ambient temp to turn on fan"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_fan_tempMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 24"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_fan_tempMAX = keyInput;
            EEPROM.updateInt(28,relay_fan_tempMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_fan_tempMAX = "));
            lcd.print(relay_fan_tempMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
        case '3': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Min tank temp to turn on heater"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_heater_tempMIN);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 10"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_heater_tempMIN = keyInput;
            EEPROM.updateInt(30,relay_heater_tempMIN);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_heater_tempMIN = "));
            lcd.print(relay_heater_tempMIN);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
        case '4': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Max tank temp to turn off heater"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_heater_tempMAX);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 14"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_heater_tempMAX = keyInput;
            EEPROM.updateInt(32,relay_heater_tempMAX);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_heater_tempMAX = "));
            lcd.print(relay_heater_tempMAX);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
      }
      break;

    case '5': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Siphon Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:vibDelay"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:interruptTimeout"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("No activity on siphon before alert [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(vibDelay);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 3600000"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInputLong = (keyInputLong * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInputLong);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            vibDelay = keyInputLong;
            EEPROM.updateLong(38,vibDelay);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("vibDelay = "));
            lcd.print(vibDelay);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInputLong = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInputLong = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Time to block interrupt during dump [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(interruptTimeout);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 600000"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInputLong = (keyInputLong * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInputLong);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            interruptTimeout = keyInputLong;
            EEPROM.updateLong(42,interruptTimeout);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("interruptTimeout = "));
            lcd.print(interruptTimeout);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInputLong = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInputLong = 0;
            settingScreen();
          }
          break;
      }
      break;

    case '6': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Feeder Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:Duration"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:Timeout"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Time to run feeder motor [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_feederDuration);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 1000"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInputLong = (keyInputLong * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInputLong);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_feederDuration = keyInputLong;
            EEPROM.updateLong(46,relay_feederDuration);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_feederDuration = "));
            lcd.print(relay_feederDuration);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInputLong = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInputLong = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Timeout between feeder triggers [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(relay_feederTimeout);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 60000"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInputLong = (keyInputLong * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInputLong);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            relay_feederTimeout = keyInputLong;
            EEPROM.updateLong(50,relay_feederTimeout);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("relay_feederTimeout = "));
            lcd.print(relay_feederTimeout);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInputLong = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInputLong = 0;
            settingScreen();
          }
          break;
      }
      break;

    case '7': //top
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("System Settings"));
      lcd.setCursor(0, 1);
      lcd.print(F("1:keypadDebounce"));
      lcd.setCursor(0, 2);
      lcd.print(F("2:timeZone"));
      lcd.setCursor(0, 3);
      lcd.print(F("3:Reset to Default"));
      lcd.setCursor(34, 3);
      lcd.print(F("D:BACK"));
      key = keypad.waitForKey();
      switch(key) {
        
        case 'D': //second
          settingScreen();
          break;
          
        case '1': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Keypad debounce [mS]"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(keypadDebounce);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = 10"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            keypadDebounce = keyInput;
            EEPROM.updateInt(34,keypadDebounce);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("keypadDebounce = "));
            lcd.print(keypadDebounce);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;

        case '2': //second
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Time zone"));
          lcd.setCursor(0, 1);
          lcd.print(F("Current = "));
          lcd.print(timeZone);
          lcd.setCursor(20, 1);
          lcd.print(F("Default = -8"));
          lcd.setCursor(0, 2);
          lcd.print(F("Enter Value = "));
          lcd.setCursor(0, 3);
          lcd.print(F("A:Set D:Cancel"));
          lcd.setCursor(13, 2);
          lcd.cursor();
          lcd.blink();
          do {
            key = keypad.waitForKey();
            lcd.noBlink();
            lcd.noCursor();
            if(key >= '0' && key <= '9') {
              keyInput = (keyInput * 10) + (key - '0');
              lcd.setCursor(13, 2);
              lcd.print(keyInput);
            }
          } while(key >= '0' && key <= '9');
          if(key == 'A') {
            timeZone = 0 - keyInput;
            EEPROM.updateInt(36,timeZone);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("timeZone = "));
            lcd.print(timeZone);
            lcd.setCursor(0, 1);
            lcd.print(F("Written to EEPROM"));
            keyInput = 0;
            delay(3000);
            settingScreen();
          } else {
            keyInput = 0;
            settingScreen();
          }
          break;
          
        case '3': //second
          resetScreen();
          break;
      }
      break;
  }
}

void resetScreen() {
  lcd.clear();
  lcd.setCursor(13, 0);
  lcd.print(F("<<<Warning>>>"));
  lcd.setCursor(0, 1);
  lcd.print(F("This will revert all settings to default"));
  lcd.setCursor(13, 2);
  lcd.print(F("<<<Warning>>>"));
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Press A to reset all settings to default"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press D to return to main menu"));
  key = keypad.waitForKey();
  if(key == 'A') {
    flowSamplePeriod = 500; // The sample period in milliseconds
    sonarDistanceMAX = 50; // Maximum sensor distance is rated at 400-500cm
    tankLevelMAX = 5; // Maximum tank level to turn off pump
    relay_light_ldrMIN = 200; // Minimum light level before the lights will turn on
    relay_light_ldrMAX = 700; // Maximum light level before the lights will turn off
    relay_light_morningMIN = 10; // Time in morning that lights will turn on
    relay_light_nightMAX = 18; // Time in evening when lights will turn off
    relay_light_tempMIN = 12; // Tank temp must be below for lights to turn on
    relay_light_tempMAX = 16; // Tank temp must be above for lights to turn off. Hysteresis
    relay_pump_morningMIN = 8; // Time in morning when pump cycle switches
    relay_pump_nightMAX = 22; // Time in evening when pump cycle switches
    relay_pump_tempMIN = 10; // Inside temp must be below for pump to turn off at night. Prevents growbeds acting as heatsink.
    relay_pump_tempMAX = 14; // Inside temp must be above for pump to turn on at night. Hysteresis.
    relay_fan_tempMIN = 25; // Air temp inside must be below to turn off fans. Hysteresis
    relay_fan_tempMAX = 27; // Air temp inside must be above to turn on fans
    relay_heater_tempMIN = 10; // Tank temp inside must be below to turn on heater
    relay_heater_tempMAX = 14; // Tank temp inside must be above to turn off heater. Hysteresis
    keypadDebounce = 10; // Keypad debounce. Default is 10mS
    timeZone = -8;
    vibDelay = 3600000; // Trigger siphon alert after no activity in 1 hour
    interruptTimeout = 600000; // Disable interrupt for timeout period while siphon dumps. 600000mS = 10 mins
    relay_feederDuration = 1000; // Duration that feeder motor is on after triggered
    relay_feederTimeout = 60000; // Duration that the feeder is inactive after a trigger.
    first_run = 1234; // Flag that reset has happened
    EEPROM.updateInt(0,flowSamplePeriod);
    EEPROM.updateInt(2,sonarDistanceMAX);
    EEPROM.updateInt(4,tankLevelMAX);
    EEPROM.updateInt(6,relay_light_ldrMIN);
    EEPROM.updateInt(8,relay_light_ldrMAX);
    EEPROM.updateInt(10,relay_light_morningMIN);
    EEPROM.updateInt(12,relay_light_nightMAX);
    EEPROM.updateInt(14,relay_light_tempMIN);
    EEPROM.updateInt(16,relay_light_tempMAX);
    EEPROM.updateInt(18,relay_pump_morningMIN);
    EEPROM.updateInt(20,relay_pump_nightMAX);
    EEPROM.updateInt(22,relay_pump_tempMIN);
    EEPROM.updateInt(24,relay_pump_tempMAX);
    EEPROM.updateInt(26,relay_fan_tempMIN);
    EEPROM.updateInt(28,relay_fan_tempMAX);
    EEPROM.updateInt(30,relay_heater_tempMIN);
    EEPROM.updateInt(32,relay_heater_tempMAX);
    EEPROM.updateInt(34,keypadDebounce);
    EEPROM.updateInt(36,timeZone);
    EEPROM.updateLong(38,vibDelay);
    EEPROM.updateLong(42,interruptTimeout);
    EEPROM.updateLong(46,relay_feederDuration);
    EEPROM.updateLong(50,relay_feederTimeout);
    EEPROM.updateInt(54,first_run);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Settings Reset to Factory Default"));
    lcd.setCursor(0, 1);
    lcd.print(F("Exiting..."));
    delay(3000);
    homeScreen();
  } else {
    homeScreen();
  }
}

void calibrationScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Press A for pH"));
  lcd.setCursor(0, 1);
  lcd.print(F("Press B for Dissolved Oxygen"));
  lcd.setCursor(0, 2);
  lcd.print(F("Press C to Reset to Factory Default"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press D to Exit"));
  key = keypad.waitForKey();
  switch(key) {
    
  case 'D':
    mainMenu();
    break;
    
  case 'A':
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Wash and put pH probe in pH 7.0 solution"));
    lcd.setCursor(0, 1);
    lcd.print(F("Push 7 button to continue"));
    key = keypad.waitForKey();
    if(key == '7') {
      Serial3.print("C"); // Continuous pH reading
      Serial3.print("\r"); // Send carriage return to end command
      calTimer = millis() + 120000;
      do {
        calRemaining = calTimer - millis();
        lcd.setCursor(0, 1);
        lcd.print(F("Time Remaining :          "));
        lcd.setCursor(16, 1);
        lcd.print(calRemaining / 1000);
      } while(calRemaining >= 0);
      Serial3.print("S"); // Calibrated pH7.0 reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("pH 7.0 Calibration Completed"));
      delay(3000);
    } else {
      Serial3.print("E"); // End continuous reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Exiting..."));
      delay(2000);
      calibrationScreen();
      break;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Wash and put pH probe in pH 4.0 solution"));
    lcd.setCursor(0, 1);
    lcd.print(F("Push 4 button to continue"));
    key = keypad.waitForKey();
    if(key == '4') {
      Serial3.print("C"); // Continuous pH reading
      Serial3.print("\r"); // Send carriage return to end command
      calTimer = millis() + 120000;
      do {
        calRemaining = calTimer - millis();
        lcd.setCursor(0, 1);
        lcd.print(F("Time Remaining :          "));
        lcd.setCursor(16, 1);
        lcd.print(calRemaining / 1000);
      } while(calRemaining >= 0);
      Serial3.print("F"); // Calibrated pH4.0 reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("pH 4.0 Calibration Completed"));
      lcd.setCursor(0, 1);
      delay(3000);
    } else {
      Serial3.print("E"); // End continuous reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Exiting..."));
      delay(2000);
      calibrationScreen();
      break;
    }
      
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Wash and put pH probe in pH 10.0 solution"));
    lcd.setCursor(0, 1);
    lcd.print(F("Push 1 button to continue"));
    key = keypad.waitForKey();
    if(key == '1') {
      Serial3.print("C"); // Continuous pH reading
      Serial3.print("\r"); // Send carriage return to end command
      calTimer = millis() + 120000;
      do {
        calRemaining = calTimer - millis();
        lcd.setCursor(0, 1);
        lcd.print(F("Time Remaining :          "));
        lcd.setCursor(16, 1);
        lcd.print(calRemaining / 1000);
      } while(calRemaining >= 0);
      Serial3.print("T"); // Calibrated pH10.0 reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("pH 10.0 Calibration Completed"));
      lcd.setCursor(0, 1);
      delay(3000);
    } else {
      Serial3.print("E"); // End continuous reading
      Serial3.print("\r"); // Send carriage return to end command
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Exiting..."));
      delay(2000);
      calibrationScreen();
      break;
    }

    Serial3.print("E"); // End continuous reading
    Serial3.print("\r"); // Send carriage return to end command
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Calibration completed"));
    delay(3000);
    calibrationScreen();
    break;
    
  case 'B': // Calibrate Dissolved Oxygen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Wet probe, flick and leave in open air"));
    lcd.setCursor(0, 1);
    lcd.print(F("Push 5 button to continue"));
    key = keypad.waitForKey();
    if(key == '5') { // Check for a 5 key.
      calTimer = millis() + 120000;
      do {
        calRemaining = calTimer - millis();
        lcd.setCursor(0, 1);
        lcd.print(F("Time Remaining :          "));
        lcd.setCursor(16, 1);
        lcd.print(calRemaining / 1000);
      } while(calRemaining >= 0);
      Serial2.print("M"); // Calibrated dissolved oxygen reading
      Serial2.print("\r"); // Send carriage return to end command
      delay(700);
      holding = 0;
      if(Serial2.available() > 1) {  
        holding = Serial2.available();
        for(i=0; i<holding; i++){        
          stamp_data_do[i]= Serial2.read();
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Dissolved Oxygen Calibration completed"));
        lcd.setCursor(0, 1);
        for(i=0; i<holding; i++){
          lcd.print(stamp_data_do[i]);
        }
        delay(3000);
        calibrationScreen();
        break;
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Dissolved Oxygen Calibration Error"));
        lcd.setCursor(1, 0);
        lcd.print(F("Please check hardware and try again"));
        delay(3000);
        calibrationScreen();
        break;
      }
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Exiting..."));
      delay(2000);
      calibrationScreen();
      break;
    }
    calibrationScreen();
    break;
    
  case 'C': // factory default
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print(F("<<<Warning>>>"));
    lcd.setCursor(0, 1);
    lcd.print(F("This will revert pH and DO to default"));
    lcd.setCursor(13, 2);
    lcd.print(F("<<<Warning>>>"));
    delay(3000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Press A to reset pH and DO to default"));
    lcd.setCursor(0, 3);
    lcd.print(F("Press D to return to main menu"));
    key = keypad.waitForKey();
    if(key == 'A') {
      Serial2.print("X"); // reset do
      Serial2.print("\r"); // Send carriage return to end command
      delay(700);
      holding = 0;
      if(Serial2.available() > 1) {
        holding = Serial2.available();
        for(i=0; i<holding; i++){        
          stamp_data_do[i]= Serial2.read();
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Dissolved Oxygen board --> "));
        lcd.setCursor(0, 1);
        for(i=0; i<holding; i++){
          lcd.print(stamp_data_do[i]);
        }
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Dissolved Oxygen reset Error"));
        lcd.setCursor(1, 0);
        lcd.print(F("Please check hardware and try again"));
        delay(3000);
        calibrationScreen();
        break;
      }
      Serial3.print("X"); // reset pH
      Serial3.print("\r"); // Send carriage return to end command
      delay(700);
      holding = 0;
      if(Serial3.available() > 1) {
        holding = Serial3.available();
        for(i=0; i<holding; i++){        
          stamp_data_ph[i]= Serial3.read();
        }
        lcd.setCursor(0, 2);
        lcd.print(F("pH board --> "));
        lcd.setCursor(0, 3);
        for(i=0; i<holding; i++){
          lcd.print(stamp_data_ph[i]);
        }
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("pH reset Error"));
        lcd.setCursor(1, 0);
        lcd.print(F("Please check hardware and try again"));
        delay(3000);
        calibrationScreen();
        break;
      }
      delay(3000);
      Serial2.print("E"); // End continuous reading
      Serial2.print("\r"); // Send carriage return to end command
      Serial3.print("E"); // End continuous reading
      Serial3.print("\r"); // Send carriage return to end command
      calibrationScreen();
      break;
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Exiting..."));
      delay(2000);
      calibrationScreen();
      break;
    }
    calibrationScreen();
    break;
  }
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void lcd2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print('0');
  }
  lcd.print(number);
}

// Demand feeder trigger interrupt service routine
void feeder() {
  relay_feederTrigger = 1;
  detachInterrupt(0);
}

// Bell siphon 1 trigger interrupt service routine
void vib1() {
  vib1Timer = millis();
  detachInterrupt(1);
}

// Bell siphon 2 trigger interrupt service routine
void vib2() {
  vib2Timer = millis();
  detachInterrupt(4);
}

// Bell siphon 3 trigger interrupt service routine
void vib3() {
  vib3Timer = millis();
  detachInterrupt(5);
}

void startEthernet() {
  //Start or restart the Ethernet connection.
  client.stop();
  Serial.println(F("Connecting Arduino to network..."));
  Serial.println();  
  delay(2000); //Wait for the connection to finish stopping
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Connection Failed, reset Arduino to try again"));
    Serial.println();
  } else {
    Serial.println(F("Arduino connected to network"));
    Serial.println();
  }
}

void syncNTP() {
  Udp.begin(localPort);
  memset(packetBuffer, 0, NTP_PACKET_SIZE); // set all bytes in the buffer to 0
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12] = 49;   // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(timeServer, 123); //NTP requests are to port 123
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket();
  delay(1000);  // wait to see if a reply is available
  if(Udp.parsePacket()) {
    Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]); // First, esxtract the two words:
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
    unsigned long secsSince1900 = highWord << 16 | lowWord; // this is NTP time (seconds since Jan 1 1900):
    const unsigned long seventyYears = 2208988800UL;  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:  
    unsigned long epoch = secsSince1900 - seventyYears + (timeZone * 3600);
    Serial.println(" ");
    Serial.print(F("NTP Epoch = "));
    Serial.print(epoch);
    Udp.stop();
    rtcEpoch = RTC.get();
    Serial.print(F(" --- Pre-Sync RTC Epoch = "));
    Serial.print(rtcEpoch);
    RTC.set(epoch);
    Serial.println(F(" RTC synced"));
  }
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
