// 
// IoT Moisture sensor v2
//
// reads capacitive soil moisture sensor and sends data to Adafruit IO cloud
// battery management with Sparkfun Fuel Gauge (MAX17043GU) and TP4056 lipo charger (1S)
// wifi via esp8266 (Wemos Di1 Mini), wifi configuration with WiFiManager
// uses esp8266 deep sleep and rtc memory to store variables while deep sleep

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

// Sparkfun Fuel Gauge configuration
#include <Wire.h>
#include <MAX17043GU.h>

#define MAX17043_ADDRESS 0x36

float voltage;
float percentage;

MAX17043GU battery;

// Adafruit IO configuration
#include "config.h"

// RTC memory read/write functions
extern "C" {
#include "user_interface.h" 
}

// analog pin for moisture data
#define MOISTURE_PIN A0

// set up feeds on Adafruit IO (must be existing feeds on io.adafruit.com)
AdafruitIO_Feed *moisture = io.feed("moisture");
AdafruitIO_Feed *voltLev = io.feed("voltage");
AdafruitIO_Feed *batLev = io.feed("battery");


// io port to power the moisture sensor on demand
// not used yet
#define SENSOR_POWER_PIN D8

// data state
int currentVal = 0; // current moisture reading
int last = -1;      // last reading from stored rtc data

// data structure for storing data in rtc memoy during deep sleep
// see https://github.com/SensorsIot/ESP8266-RTC-Memory 
typedef struct {
  int last;
  int counter;
} rtcStore;

rtcStore rtcMem;

// sleep for this many seconds
const int sleepSeconds = 60;


void setup() {

  // configure io pins
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  
  // start the serial connection
  Serial.begin(115200);

  // Start I2C
  Wire.begin();

  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.autoConnect("Blumentopf");
  
  // wait for serial monitor to open
  while(! Serial);

  Serial.println("Connected to Wifi");

  // connecting to io.adafruit.com
  Serial.println("Now connecting to Adafruit IO");
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

  // io.run(); is required at the top of the loop
  // it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  // read stored value from rtc memory
  // parameter: address (starts at 64), variable name, size in byte (4 byte steps)
  system_rtc_mem_read(64, &rtcMem, sizeof(rtcMem));
  Serial.print("Counter: "); // how many times 
  Serial.println(rtcMem.counter);
  Serial.print("Last reading -> ");
  Serial.println(rtcMem.last);
  
  // start battery monitoring
  battery.restart();
  voltage = battery.voltageLevel();
  percentage = battery.fuelLevel();

  Serial.print("Battery voltage: ");
  Serial.print(voltage, 2);
  Serial.println("V");
  Serial.print("Battery level: ");
  Serial.print(percentage, 2);
  Serial.println("%");

  // save battery levels to io
  voltLev->save(voltage);
  batLev->save(percentage);

  // enabling sensor power
  // not used
  // digitalWrite(SENSOR_POWER_PIN,HIGH);
  // delay(200);

  // grab the current state of the moisture sensor
  Serial.println("Reading moisture -> ");
  
  // currentVal = analogRead(MOISTURE_PIN);
  // calibration: 400 -> sensor in water; 900 -> sensor in air 
  currentVal = map(analogRead(MOISTURE_PIN), 400, 900, 100, 0);
  delay(100);

  // shutting down sensor power
  // digitalWrite(SENSOR_POWER_PIN,LOW);
  
  // do nothing if the value hasn't changed since last measurement
  if(currentVal == rtcMem.last) {
    // set esp to sleep
    Serial.printf("Nothing changed - going to sleep for %d seconds\n\n", sleepSeconds);
    }
  else {
    // save the current state to the moisture feed to Adafruit IO
    digitalWrite(BUILTIN_LED,LOW);
    Serial.print("Sending new data to io -> ");
    Serial.print("Feuchte: ");
    Serial.println(currentVal);
    moisture->save(currentVal);
    delay(500);
    digitalWrite(BUILTIN_LED,HIGH);
    Serial.printf("Going to sleep for %d seconds\n\n", sleepSeconds);
    }
  
  // store last state and counter in rtc memory
  rtcMem.last = currentVal;
  rtcMem.counter = rtcMem.counter + 1;
  // rtcMem.counter = 0; // uncomment to reset couter
  if (rtcMem.counter < 0) {
    rtcMem.counter = 0;
  }
  system_rtc_mem_write(64, &rtcMem, sizeof(rtcMem));
  
  // sending esp to sleep
  ESP.deepSleep(sleepSeconds * 1000000);
  
}

void loop() {
  
}
