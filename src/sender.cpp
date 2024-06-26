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
#define MAX_LUM 28
#define MID_LUM 6
#define MIN_LUM 1
#define MAX_EVENT_CNT 27

uint8_t receiver_mac[] = {0x08, 0xB6, 0x1F, 0xB9, 0x2B, 0xE4};

bool eventFlag = false;
bool start = LOW;
bool state = LOW;
bool prevState = LOW;
uint8_t i = 0;
uint8_t data[2];
uint8_t lum = MIN_LUM;
uint8_t delayTime = 1;
uint8_t lumArr[MAX_EVENT_CNT]  = {0,       MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, 15,      MID_LUM,
                                  MIN_LUM, MID_LUM, MAX_LUM, 12,      MIN_LUM, 10,      10,      22,      MID_LUM, 3,       MID_LUM, 12,
                                  22,      MID_LUM, 0};
uint16_t timeArr[MAX_EVENT_CNT] = {2,       20,      61,     108,     137,     157,     190,     235,     267,     322,     358,     408,
                                  428,     448,     482,     524,     586,     625,     626,     668,     701,     729,     745,     804,
                                  824,     886,     956};

// Monitoring
unsigned long lastMonitorTime = 0;
unsigned long MonitorDelay = 1000;

// delayTime calculation is dependent on the dimmer resolution, at 13 bits (2^13=8192) 1% is ~82 steps, so:
// delayTime = (duration / percent_change) / 82
// example: for 60 seconds and 34% change, (60 / 34) / 82 = 21 milliseconds
// Other way around: delayTime = ((duration * 100) / (percent_change * 2^13))
uint8_t DelayCalc(uint8_t duration, uint8_t percent_change) {
    float delayTimeMillis = (1000 * ((float)duration * 100) / ((float)percent_change * pow(2, 13)));
    // Serial.println("Delay time in milliseconds: " + String(delayTimeMillis));
    return (uint8_t)round(delayTimeMillis);
}


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

void loop() {
    start = digitalRead(START_INPUT_PIN); // read state from start pin

    state = digitalRead(QUE_INPUT_PIN); // read state from que pin
    
    if (state != prevState) {
        Serial.println("State change detected.");
        i++;
        Serial.println("Event count: " + String(i));
        lum = lumArr[i];
        uint8_t duration = timeArr[i] - timeArr[i-1];
        uint8_t percent_change = (uint8_t)(abs((int8_t)lumArr[i] - (int8_t)lumArr[i-1]));
        Serial.println("Duration is: " + String(duration));
        Serial.println("Percent change is: " + String(percent_change));
        delayTime = DelayCalc(duration, percent_change);
        data[0] = lum;
        data[1] = delayTime;
        ESPNow.send_message(receiver_mac, data, 2);
    }

    if ((i >= MAX_EVENT_CNT) || (start == HIGH)) {
        Serial.println("Event count reached maximum or start IO is high. restarting.");
        i = 0;
        data[0] = MIN_LUM;
        data[1] = 1;
        ESPNow.send_message(receiver_mac, data, 2);
        Serial.println("sent lum is: " + String(MIN_LUM));
        Serial.println("sent delay is: " + String(1));
        Serial.println("Event count: " + String(i));
    }

    // Update previous state
    prevState = state;

    // Monitoring, not a must, just for debugging
    if((millis() - lastMonitorTime) > MonitorDelay) {
        Serial.print("ESP Now sender event count: "); Serial.println(i);
        lastMonitorTime = millis();
    }

    // Delay to slow things down
    delay(1);
}