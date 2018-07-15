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


// Adafruit IO configuration
#include "config.h"

// analog pin for moisture data
#define MOISTURE_PIN A0

// set up the 'moisture' feed on Adafruit IO (must be configured there as well)
AdafruitIO_Feed *moisture = io.feed("moisture");

// moisture data
int current = 0;

// sleep for this many seconds
const int sleepSeconds = 600;

void setup() {

  pinMode(BUILTIN_LED, OUTPUT);
  
  // start the serial connection
  Serial.begin(115200);

  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.autoConnect("Blumentopf");
  
  // wait for serial monitor to open
  while(! Serial);

  Serial.println("Connected to Wifi");

  // connect to io.adafruit.com
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

  // io.run(); is required  at the top of the loop
  // it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  Serial.println("reading moisture -> ");
  current = analogRead(MOISTURE_PIN);
    
  // Calibration required for sensor type
  //current = map(analogRead(MOISTURE_PIN), 400, 900, 100, 0);
    
  Serial.print(current);

  // save the current state to the moisture feed to Adafruit IO
  digitalWrite(BUILTIN_LED,LOW);
  Serial.print("sending new data to io -> ");
  Serial.println(current);
  moisture->save(current);
  delay(500);
  digitalWrite(BUILTIN_LED,HIGH);
  Serial.printf("Going to sleep for %d seconds\n\n", sleepSeconds);
   
  // set esp to sleep
  ESP.deepSleep(sleepSeconds * 1000000);
}

void loop() {
  // no loop because esp is 
  
}

