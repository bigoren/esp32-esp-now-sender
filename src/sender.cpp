/*
 * "THE BEER-WARE LICENSE" (Revision 42):
 * regenbogencode@gmail.com wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me lum beer in return
 */
#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif ESP32
#include <WiFi.h>
#endif
#include "ESPNowW.h"

#define START_INPUT_PIN 2
#define QUE_INPUT_PIN 4
#define MAX_LUM 35
#define MIN_LUM 1
#define MAX_EVENT_CNT 15

// uint8_t receiver_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
// uint8_t receiver_mac[] = {0x40, 0x22, 0xD8, 0x5E, 0x6D, 0x04};
uint8_t receiver_mac[] = {0x08, 0xB6, 0x1F, 0xB9, 0x2B, 0xE4};

bool eventFlag = false;
bool start = LOW;
bool state = LOW;
bool prevState = LOW;
uint8_t eventCount = 0;
uint8_t data[2];
uint8_t lum = MIN_LUM;
uint8_t delayTime = 1;

// Monitoring
unsigned long lastMonitorTime = 0;
unsigned long MonitorDelay = 1000;

void setup() {
    Serial.begin(115200);
    Serial.println("ESPNow sender started.");
#ifdef ESP8266
    WiFi.mode(WIFI_STA); // MUST NOT BE WIFI_MODE_NULL
#elif ESP32
    WiFi.mode(WIFI_MODE_STA);
#endif
    WiFi.disconnect();
    ESPNow.init();
    ESPNow.add_peer(receiver_mac);

    pinMode(START_INPUT_PIN, INPUT);
    pinMode(QUE_INPUT_PIN, INPUT);
    Serial.println("Start pin number is: " + String(START_INPUT_PIN));
    Serial.println("Que pin number is: " + String(QUE_INPUT_PIN));

    ESPNow.send_message(receiver_mac, data, 2);
    Serial.println("Initial lum is: " + String(lum));
    Serial.println("Default delay is: " + String(delayTime));
}

// delayTime calculation is dependent on the dimmer resolution, at 13 bits (2^13=8192) 1% is ~82 steps, so:
// delayTime = (duration / percent_change) / 82
// example: for 60 seconds and 34% change, (60 / 34) / 82 = 21 milliseconds

void loop() {
    start = digitalRead(START_INPUT_PIN); // read state from start pin

    state = digitalRead(QUE_INPUT_PIN); // read state from que pin
    // Set direction when state changes
    if ((state == HIGH) && (prevState == LOW)) {
        eventFlag = true;
        eventCount++;
        Serial.println("State change from LOW to HIGH");
        if (eventCount == 1) {
            lum = MAX_LUM;
            delayTime = 21; // for 59 secs and 34% change
        } else if (eventCount == 3) {
            lum = MAX_LUM;
            delayTime = 19; // for 53 secs and 34% change
        } else if (eventCount == 5) {
            lum = MAX_LUM;
            delayTime = 21;
        } else if (eventCount == 7) {
            lum = MAX_LUM;
            delayTime = 19;
        } else if (eventCount == 9) {
            lum = MAX_LUM;
            delayTime = 25; // for 70 secs and 34% change
        } else if (eventCount == 11) {
            lum = MAX_LUM;
            delayTime = 16; // for 45 secs and 34% change 
        } else if (eventCount == 13) {
            lum = MAX_LUM;
            delayTime = 23; // for 64 secs and 34% change
        }
    } else if ((state == LOW) && (prevState == HIGH)) {
        eventFlag = true;
        eventCount++;
        Serial.println("State change from HIGH to LOW");
        if (eventCount == 2) {
            lum = MIN_LUM;
            delayTime = 27; // for 76 secs and 34% change
        } else if (eventCount == 4) {
            lum = MIN_LUM;
            delayTime = 27;
        } else if (eventCount == 6) {
            lum = MIN_LUM;
            delayTime = 35; // for 100 secs and 34% change
        } else if (eventCount == 8) {
            lum = MIN_LUM;
            delayTime = 35;
        } else if (eventCount == 10) {
            lum = MIN_LUM;
            delayTime = 21;
        } else if (eventCount == 12) {
            lum = MIN_LUM;
            delayTime = 13; // for 37 secs and 34% change
        } else if (eventCount == 14) {
            lum = MIN_LUM;
            delayTime = 30; // for 85 secs and 34% change
        }
    }
    // Send data at eventFlag
    if (eventFlag) {
        eventFlag = false;
        data[0] = lum;
        data[1] = delayTime;
        ESPNow.send_message(receiver_mac, data, 2);
        Serial.println("sent lum is: " + String(lum));
        Serial.println("sent delay is: " + String(delayTime));
        Serial.println("Event count: " + String(eventCount));
    }

    if ((eventCount >= MAX_EVENT_CNT) || (start == HIGH)) {
        Serial.println("Event count reached maximum or start IO is high. restarting.");
        eventCount = 0;
        data[0] = MIN_LUM;
        data[1] = 1;
        ESPNow.send_message(receiver_mac, data, 2);
        Serial.println("sent lum is: " + String(MIN_LUM));
        Serial.println("sent delay is: " + String(1));
        Serial.println("Event count: " + String(eventCount));
    }

    // Update previous state
    prevState = state;

    // Monitoring, not a must, just for debugging
    if((millis() - lastMonitorTime) > MonitorDelay) {
        Serial.print("ESP Now sender event count: "); Serial.println(eventCount);
        lastMonitorTime = millis();
    }
    // Delay to slow things down
    delay(1);
}