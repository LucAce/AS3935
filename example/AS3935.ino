//*****************************************************************************
// Copyright (c) 2019 LucAce
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//*****************************************************************************
//
// Arduino I2C AMS AS3935 Library Example
//
// Board Compatibility:
// - Arduino Uno R3
//     https://store.arduino.cc/usa/arduino-uno-rev3/
// - WEMOS D1 Mini
//     https://wiki.wemos.cc/products:d1:d1_mini
// - Playing With Fusion Lightning Sensor Breakout (I2C Configuration)
//     https://www.playingwithfusion.com/productview.php?pdid=22
// - Qwiic AS3935 Lightning Detector Breakout (I2C Pinout)
//     https://www.sparkfun.com/products/15057
//
// Arduino Library Dependencies:
// - Wire
// - ESP8266WiFi (If using ESP8266 device)
//
// Boards Package (If using ESP8266):
// - http://arduino.esp8266.com/stable/package_esp8266com_index.json
//
// Notes:
// - This example uses IRQ Pin Polling to determine when an interrupt
//   on the AS3935 occurs.
//
//*****************************************************************************

#include <Wire.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif

#include "AS3935.h"

// Serial port baud rate
#define SERIAL_BAUD_RATE 115200

// LED On/Off States
#ifdef ESP8266
#define LED_ON  0
#define LED_OFF 1
#else
#define LED_ON  1
#define LED_OFF 0
#endif

// AS3935 I2C Address
#define AS3935_I2C_ADDR 0x03

// AS3935 Tune the Antenna
// 1: Use the tuneAntenna function to find value
// 0: Use the AS3935_TUNING_CAP value
#define AS3935_TUNE_ANTENNA 1

// AS3935 Tuning Capacitor Value (in pf)
// Valid Range 0pf to 120pf in step of 8pf
#define AS3935_TUNING_CAP 56

// AS3935 Noise Floor
// 0x00 to 0x07, default of 0x02
#define AS3935_NOISE_FLOOR 0x02

// AS3935 Interrupt Input Pin
#define AS3935_IRQ_PIN D5

// AS3935 Indoor or Outdoor
// 1: Use Indoor AFE Gain Boost
// 0: Use Outdoor AFE Gain Boost
#define AS3935_INDOOR 1

// AS3935 Interrupt Register Values
#define AS3935_NOISE_INT     0x01
#define AS3935_DISTURBER_INT 0x04
#define AS3935_LIGHTNING_INT 0x08

// AS3935 I2C Interface
AS3935 as3935;


//*****************************************************************************
// Function: setup
// Configuration setup
//*****************************************************************************
void setup() {
    // Define IO
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(AS3935_IRQ_PIN, INPUT);

    // For this example, turn WIFI off to reduce noise
    #ifdef ESP8266
    WiFi.mode(WIFI_OFF);
    #endif

    // Turn LED On
    digitalWrite(LED_BUILTIN, LED_ON);

    // Enable Serial Port
    Serial.begin(SERIAL_BAUD_RATE);
    delay(100);
    while (!Serial);

    Serial.println();
    Serial.println(F("Arduino I2C AMS AS3935 Library Example"));

    // Enable AS3935 Sensor
    Serial.println();
    Serial.println(F("Enabling AS3935 Sensor"));
    if (!as3935.begin(AS3935_I2C_ADDR)) {
        Serial.println();
        Serial.println(F("Error: AS3935 Sensor Not Found"));
        Serial.print(F("AS3935 I2C Address: 0x"));
        Serial.println(AS3935_I2C_ADDR, HEX);
        // Force Watch Dog Timeout and Restart
        while(1);
    }

    // AS3935 Sensor Boot Up Delay
    delay(10);

    #if AS3935_TUNE_ANTENNA == 1
    // Find the Tuning Capacitor value
    as3935.tuneAntenna(AS3935_IRQ_PIN);
    #else
    // Manually set Tuning Capacitor value
    as3935.setTuningCapacitor(AS3935_TUNING_CAP);
    #endif

    Serial.print(F("AS3935 Tuning Capacitor: "));
    Serial.print(as3935.getTuningCapacitor(), DEC);
    Serial.println(" pf");

    // Set Indoor/Outdoor
    #if AS3935_INDOOR == 1
    Serial.println(F("AS3935 Gain Boost: Indoor"));
    as3935.setAfeGainBoostIndoor();
    #else
    Serial.println(F("AS3935 Gain Boost: Outdoor"));
    as3935.setAfeGainBoostOutdoor();
    #endif

    // Set the noise floor level
    as3935.setNoiseFloorLevel(AS3935_NOISE_FLOOR);
    Serial.print(F("AS3935 Noise Floor: "));
    Serial.println(as3935.getNoiseFloorLevel(), DEC);
    Serial.println();

    // Turn LED Of
    digitalWrite(LED_BUILTIN, LED_OFF);
}


//*****************************************************************************
// Function: loop
// Main Processing Loop.
//*****************************************************************************
void loop() {
    uint8_t  irq;
    uint8_t  distance;

    // Return if no interrupt
    if (digitalRead(AS3935_IRQ_PIN) == LOW)
        return;

    digitalWrite(LED_BUILTIN, LED_ON);

    // Delay at least 2ms following interrupt to for registers to populate
    delay(2);

    // Get interrupt register value
    irq = as3935.getInterrupt();

    switch (irq) {
        case AS3935_NOISE_INT:
            Serial.println(F("AS3935 Noise Interrupt Detected"));
            break;
        case AS3935_DISTURBER_INT:
            Serial.println(F("AS3935 Disturber Interrupt Detected"));
            break;
        case AS3935_LIGHTNING_INT:
            Serial.println(F("AS3935 Lightning Interrupt Detected"));
            distance = as3935.getDistance();
            Serial.print(F("AS3935 Lightning Distance: "));
            Serial.print(distance, DEC);
            Serial.println(F(" km"));
            break;
        default:
            Serial.println(F("AS3935 Invalid Interrupt Detected"));
            break;
    }

    delay(500);
    digitalWrite(LED_BUILTIN, LED_OFF);
}
