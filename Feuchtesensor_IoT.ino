// 
// IoT Moisture sensor v1
//
// based on
// Adafruit IO Analog In Example
// Tutorial Link: https://learn.adafruit.com/adafruit-io-basics-analog-input
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <Adafruit_INA219.h>

// Adafruit IO configuration
#include "config.h"

// this is for the RTC memory read/write functions
extern "C" {
#include "user_interface.h" 
}

// analog pin for moisture data
#define MOISTURE_PIN A0

// set up the 'moisture' feed on Adafruit IO (must be configured there as well)
AdafruitIO_Feed *moisture = io.feed("moisture");

AdafruitIO_Feed *power = io.feed("power");
AdafruitIO_Feed *current = io.feed("current");

// INA sensor
Adafruit_INA219 ina219;

// Measurement variables
float current_mA;
float power_mW;
float loadvoltage_V;


// io port to power the moisture sensor on demand
#define SENSOR_POWER_PIN D8

// data state
int currentVal = 0;
int median = 0;
int samples = 5;
int last = -1;

// data structure for storing data in rtc memoy during deep sleep
// see https://github.com/SensorsIot/ESP8266-RTC-Memory 
typedef struct {
  int last;
  int counter;
} rtcStore;

rtcStore rtcMem;

// sleep for this many seconds
const int sleepSeconds = 600;


void setup() {

  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  
  // start the serial connection
  Serial.begin(115200);

  // Init INA219
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.autoConnect("AutoConnectAP");
  
  // wait for serial monitor to open
  while(! Serial);

  Serial.println("Connected to Wifi");

  Serial.println("Now connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    digitalWrite(BUILTIN_LED,LOW);
    delay(500);
    digitalWrite(BUILTIN_LED,HIGH);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  digitalWrite(BUILTIN_LED,HIGH);

  // io.run(); is required  at the top of the loop
  // it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  // read stored value from rtc memory
  // parameter: address (starts at 64), variable name, size in byte (4 byte steps)
  system_rtc_mem_read(64, &rtcMem, sizeof(rtcMem));
  Serial.print("Counter: ");
  Serial.println(rtcMem.counter);
  Serial.print("Last reading -> ");
  Serial.println(rtcMem.last);

  current_mA = measureCurrent();
  power_mW = measurePower();
  loadvoltage_V = measureVoltage();

  Serial.print("reading current: ");
  Serial.println(current_mA);
  Serial.print("reading voltage: ");
  Serial.println(loadvoltage_V);

  // enabling sensor power
  digitalWrite(SENSOR_POWER_PIN,HIGH);
  delay(200);

  Serial.println("reading moisture -> ");
  // grab the current state of the moisture sensor
  // taking several readings and calculate median

  for (int i = 0; i < samples; i++) {
    //currentVal = analogRead(MOISTURE_PIN);
    currentVal = map(analogRead(MOISTURE_PIN), 400, 900, 100, 0);
    Serial.print(i);
    Serial.print(": ");
    Serial.print(currentVal);
    median = median + currentVal;
    
    Serial.print(" | ");
    Serial.println(median);
    delay(100);
  }

  //shutting down sensor power
  digitalWrite(SENSOR_POWER_PIN,LOW);
  
  median = median / samples;
  Serial.print("Median: ");
  Serial.println(median);

  // do nothing if the value hasn't changed since last measurement
  if(median == rtcMem.last) {
    // set esp to sleep
    Serial.printf("Nothing changed - going to sleep for %d seconds\n\n", sleepSeconds);
    }
  else {
    // save the current state to the moisture feed to Adafruit IO
    digitalWrite(BUILTIN_LED,LOW);
    Serial.print("sending new data to io -> ");
    Serial.println(median);
    moisture->save(median);
    current->save(current_mA);
    power->save(loadvoltage_V);
    delay(500);
    digitalWrite(BUILTIN_LED,HIGH);
    Serial.printf("Going to sleep for %d seconds\n\n", sleepSeconds);
    }
  
  // store last state and counter in rtc memory
  rtcMem.last = median;
  rtcMem.counter = rtcMem.counter + 1;
  // rtcMem.counter = 0;
  if (rtcMem.counter < 0) {
    rtcMem.counter = 0;
  }
  system_rtc_mem_write(64, &rtcMem, sizeof(rtcMem));
  
  // set esp to sleep
  ESP.deepSleep(sleepSeconds * 1000000);
  
}

void loop() {
  
}

// Function to measure current
float measureVoltage() {

  // Measure
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage_V = busvoltage + (shuntvoltage / 1000);
  
  // If negative, set to zero
  if (loadvoltage_V < 0) {
    loadvoltage_V = 0.0; 
  }
 
  return loadvoltage_V;
}

// Function to measure current
float measureCurrent() {

  // Measure
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");

  // If negative, set to zero
  if (current_mA < 0) {
    current_mA = 0.0; 
  }
 
  return current_mA;
  
}

float measurePower() {

  // Measure
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");

  // If negative, set to zero
  if (current_mA < 0) {
    current_mA = 0.0; 
  }
 
  return current_mA * loadvoltage;
  
}
