/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
**/


#include "sha204_library.h"
#include "sha204_lib_return_codes.h"

/** @brief Make use of the MySensors framework without invoking the entire system */
//#define MY_CORE_ONLY


// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_DEBUG_VERBOSE_RFM69

// The switch Node ID
//#define MY_NODE_ID 0x43


/* Each Button status (On or Off) can be sent to a different Relay or Actuator NodeId address.  
 *  relayNodeID stores Relay or Actuator NodeId addresses. Each adress can have different Child\Sensor ID.  
 *  relayChildID is Child\Sensor ID's array. 
 * int relayNodeID[4] = {0xF2, 0xA0,0x0}; 
 * int relayChildID[4] = {1, 2, NULL};
 * above declaration means: Button 1 will send it state to assigned Relay Address 0xF2 with Child\Sensor ID 1. 
 * Button 2 will send it state to assigned Relay Address 0xA0 with Child\Sensor ID 2. 
 * Button 3 have no attached sensorsors and will not be "presented" since there is NULL value.
 * NULL value indicates no switch attached to the corresponding JST connector
*/
int relayNodeID = 0x0; // Is the recepient address

// Avoid battery drain if Gateway disconnected and the node sends more than MY_TRANSPORT_STATE_RETRIES times message.
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_PARENT_NODE_IS_STATIC
#define MY_PARENT_NODE_ID 0


// Enable and select radio type attached

//#define MY_RADIO_NRF24
//#define MY_RF24_IRQ_PIN 2

#define MY_RADIO_RFM69

// if you use MySensors 2.0 use this style 
//#define MY_RFM69_FREQUENCY   RFM69_433MHZ
//#define MY_RFM69_FREQUENCY   RF69_868MHZ
#define MY_RFM69_FREQUENCY   RFM69_915MHZ


//#define MY_RFM69_NEW_DRIVER

// Enable Crypto Authentication to secure the node
//#define MY_SIGNING_ATSHA204
//#define  MY_SIGNING_REQUEST_SIGNATURES

#include <MySensors.h>


#define ALARM_INTERUPT_PIN_ON 0  // GPIO value to write to turn on attached relay
#define ALARM_INTERUPT_PIN_OFF 1 // GPIO value to write to turn off attached relay
#define ALARM_INTERUPT_PIN1_ON 0  // GPIO value to write to turn on attached relay
#define ALARM_INTERUPT_PIN1_OFF 1 // GPIO value to write to turn off attached relay

static void print_hex_buffer(uint8_t* data, size_t sz);
static bool get_atsha204a_serial(uint8_t* data);
//uint8_t buffer[SIZE_SIGNING_SOFT_HMAC_KEY + SIZE_RF_ENCRYPTION_AES_KEY + SIZE_SIGNING_SOFT_SERIAL];
uint8_t buffer[SHA204_RSP_SIZE_MAX];
const int sha204Pin = MY_SIGNING_ATSHA204_PIN; //!< The IO pin to use for ATSHA204A
static uint8_t rx_buffer[18]; //SHA204_RSP_SIZE_MAX
atsha204Class sha204(sha204Pin);

#define SPIFLASH_BLOCKERASE_32K   0xD8
#define SPIFLASH_CHIPERASE        0x60

#define SKETCH_NAME "Door\Window Sensor "
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"

#define SENSOR_INTERUPT_PIN 3
#define ALARM_INTERUPT_PIN A1
#define GREEN_LED 5
#define RED_LED 6

int BATTERY_SENSE_PIN = A6;  // select the input pin for the battery sense point
int oldBatteryPcnt = 0;


MyMessage msgSensorState(1, V_LIGHT);
MyMessage msgAlarmState(2, V_LIGHT);

char SHA204serial[19];  //SHA204_RSP_SIZE_MAX*2
  
void stringFromHexBuffer(uint8_t* data, size_t sz)
{ int j = 0;  uint8_t h=0;
  for (size_t i=0; i<sz; i++) {
    sprintf(&SHA204serial[i*2],"%02x",data[i]);
  }
  SHA204serial[18] = '\0';
}

static bool get_atsha204a_serial()
{
  uint8_t ret_code = sha204.getSerialNumber(rx_buffer);
  if (ret_code != SHA204_SUCCESS) {
    return false;
  } else {
    for (size_t i=0; i<9; i++) Serial.print(rx_buffer[i],HEX);
    return true;
  }
}


// A0    PCINT8  (PCMSK1 / PCIF1 / PCIE1)
// A1    PCINT9  (PCMSK1 / PCIF1 / PCIE1)

void AlarmIntEnable() 
{
  // Enable pin change for A0 to A1 
  //PCMSK1 |= bit (PCINT8);  
  PCMSK1 |= bit (PCINT9);  
  PCIFR  |= bit (PCIF1);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE1);   // enable pin change interrupts for A0 to A1 
}

void AlarmIntDisable() 
{
  // Disable pin change for A0 to A1 
  PCICR  ^= bit (PCIE1);   // disable pin change interrupts for A0 to A1 
  //PCMSK1 = 0x00;   
}

volatile uint8_t flagPcint = 0;

ISR (PCINT1_vect) 
{
  flagPcint = 1; 
} 

//-----------------------------------
#define CLEAR_EEPROM_VIA_RST_OFFSET 10
bool isClearEepromViaRstFlagErased = false;
void eraseClearEepromViaRstFlag(){
  if (!isClearEepromViaRstFlagErased){
    Serial.println("eraseClearEepromViaRstFlag");
    hwWriteConfig(EEPROM_LOCAL_CONFIG_ADDRESS + CLEAR_EEPROM_VIA_RST_OFFSET,0xFF);
    isClearEepromViaRstFlagErased = true;
  }
}

bool checkClearEeppromViaRstFlag(){
  //check rst source
  //if (!(MCUSR & EXTRF)){
  //  return false;
  //}
  uint8_t flagStatus = hwReadConfig(EEPROM_LOCAL_CONFIG_ADDRESS + CLEAR_EEPROM_VIA_RST_OFFSET);\
  if (flagStatus == 2) {
    return true; //affecting eeprom clear
  }
  if (flagStatus == 1){
    flagStatus++;
  } else {
    flagStatus = 1;
  }
  hwWriteConfig(EEPROM_LOCAL_CONFIG_ADDRESS + CLEAR_EEPROM_VIA_RST_OFFSET, flagStatus);
  return false;
}
//-------------------------------------

void before()
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  blinkGreenSensorLed();

  
  //analogReference(DEFAULT); // DEFAULT
  while(!Serial);
  #ifdef  MY_RADIO_RFM69
    /*  RFM reset pin is 9
     *  A manual reset of the RFM69HCW\CW is possible even for applications in which VDD cannot be physically disconnected.
     *  Pin RESET should be pulled high for a hundred microseconds, and then released. The user should then wait for 5 ms
     *  before using the module.
     */
    pinMode(9, OUTPUT);
    //reset RFM module
    digitalWrite(9, 1);
    delay(1);
    // set Pin 9 to high impedance
    pinMode(9, INPUT);
    delay(10);
  #endif

  if (!get_atsha204a_serial()) {
      memset(rx_buffer, 0xFF, 9);
      Serial.println(F("FAILED to get sha204 serial number!"));
     }
  stringFromHexBuffer(rx_buffer,9);
  Serial.print("SHA204serial: "); Serial.println(SHA204serial);

  // Setup the pins
  pinMode(ALARM_INTERUPT_PIN, INPUT);
  digitalWrite(ALARM_INTERUPT_PIN, HIGH);

  AlarmIntEnable();

  bool clearEeprom = checkClearEeppromViaRstFlag();
  if (clearEeprom){
    //EEPROM CLEAR HERE
    eraseClearEepromViaRstFlag();
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
    Serial.println("NODE ID IS CLEARED");
    wait(1000);
    hwReboot();
  }
}


void blinkGreenSensorLed(){
  digitalWrite(GREEN_LED, HIGH);
  wait(50);
  digitalWrite(GREEN_LED, LOW);
}
void blinkRedSensorLed(){
  digitalWrite(RED_LED, HIGH);
  wait(50);
  digitalWrite(RED_LED, LOW);
}
void blinkSensorLedOK(){
  for (int i = 0; i < 1; i++) {
    digitalWrite(GREEN_LED, HIGH);
    wait(30);
    digitalWrite(GREEN_LED, LOW);
    wait(30);
  }
}
void blinkSensorLedFail(){
  for (int i = 0; i < 4; i++) {
    digitalWrite(RED_LED, HIGH);
    wait(30);
    digitalWrite(RED_LED, LOW);
    wait(30);
  }
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register binary input sensor to sensor_node (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.

  present(1, S_LIGHT);
  present(2, S_LIGHT);
}

void setup() {

}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference

  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

static uint8_t prevAlarmInterruptPin = ALARM_INTERUPT_PIN_OFF; 
static uint8_t prevSensorPin = 0;
static uint8_t AlarmInterruptPin,AlarmInterruptPin1,sensorPin; 

#define MESSAGE_LENGTH 30
char msg[MESSAGE_LENGTH];


void loop(){ 
  

  if (!isClearEepromViaRstFlagErased){
    if (millis() > 5000){
      eraseClearEepromViaRstFlag();
      isClearEepromViaRstFlagErased = true;
    }
  }
  
 //   Serial.print("flagPcint: ");
 //   Serial.println(flagPcint);

 
  if ((flagPcint == 1) && isClearEepromViaRstFlagErased) {
    flagPcint = 0;

    //AlarmInterruptPin1 = debouncerALARM_INTERUPT_PIN1.read();
    //Serial.print("A0: ");
    //Serial.println(digitalRead(ALARM_INTERUPT_PIN));
    wait(50);
    AlarmInterruptPin = digitalRead(ALARM_INTERUPT_PIN);
    if (AlarmInterruptPin != prevAlarmInterruptPin){
      sprintf(msg,"%i;%s",(AlarmInterruptPin == ALARM_INTERUPT_PIN_ON)?1:0,SHA204serial);
      msgAlarmState.setDestination(0);
      send(msgAlarmState.set(msg), true);
      wait(100); 
      prevAlarmInterruptPin = AlarmInterruptPin;
    }
  } else if (isClearEepromViaRstFlagErased)  {
    sensorPin = digitalRead(SENSOR_INTERUPT_PIN); 
//    if (prevSensorPin != sensorPin){
      sprintf(msg,"%i;%s",sensorPin,SHA204serial); //\/0
      msgSensorState.setDestination(0);
      send(msgSensorState.set(msg), true);
      wait(100);  
      prevSensorPin = sensorPin;
 //   }
  }
  
  //AlarmIntEnable();

  /* Get battery level in mV
   *  2.4V - lowest level, 3v - max level
   */
  int sensorValue = readVcc();
  int batteryPcnt = 100 * (sensorValue - 2400) / 600;

  if (millis() > 6000){
  batteryPcnt > 10 ? blinkGreenSensorLed(): blinkRedSensorLed(); 
  }
    if (isClearEepromViaRstFlagErased && (sensorPin == digitalRead(SENSOR_INTERUPT_PIN)) && flagPcint == 0) {
      sleep(SENSOR_INTERUPT_PIN - 2, CHANGE, 0); //FALLING
    }
}