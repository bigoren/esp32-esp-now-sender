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
#define MID_LUM 6
#define MIN_LUM 1
#define MAX_EVENT_CNT 29

uint8_t receiver_mac[] = {0x08, 0xB6, 0x1F, 0xB9, 0x2B, 0xE4};

bool eventFlag = false;
bool start = LOW;
bool state = LOW;
bool prevState = LOW;
uint8_t i = 0;
uint8_t data[2];
uint8_t lum = MIN_LUM;
uint8_t delayTime = 1;
uint8_t lumArr[MAX_EVENT_CNT]  = {MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, MAX_LUM, MID_LUM,
                                  MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM, MID_LUM, MAX_LUM, MID_LUM,
                                  MIN_LUM, MID_LUM, MAX_LUM, MID_LUM, MIN_LUM};
uint8_t timeArr[MAX_EVENT_CNT] = {2,       30,      61,      118,     137,     167,     190,     247,     267,     287,     328,     408,
                                  428,     448,     482,     557,     577,     597,     647,     687,     707,     727,     752,     779,
                                  789,     809,     853,     918,     938};

// Monitoring
unsigned long lastMonitorTime = 0;
unsigned long MonitorDelay = 1000;

// delayTime calculation is dependent on the dimmer resolution, at 13 bits (2^13=8192) 1% is ~82 steps, so:
// delayTime = (duration / percent_change) / 82
// example: for 60 seconds and 34% change, (60 / 34) / 82 = 21 milliseconds
// Other way around: delayTime = ((duration * 100) / (percent_change * 2^13))
uint8_t DelayCalc(uint8_t duration, uint8_t percent_change) {
    return (uint8_t)(((float)duration * 100) / ((float)percent_change * (2^13)));
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
        lum = lumArr[i];
        uint8_t duration = timeArr[i] - timeArr[i-1];
        uint8_t percent_change = (uint8_t)(abs((int8_t)lumArr[i] - (int8_t)lumArr[i-1]));
        delayTime = DelayCalc(duration, percent_change);
        data[0] = lum;
        data[1] = delayTime;
        ESPNow.send_message(receiver_mac, data, 2);
        Serial.println("sent lum is: " + String(lum));
        Serial.println("sent delay is: " + String(delayTime));
        Serial.println("Event count: " + String(i));
    }

    // if ((state == HIGH) && (prevState == LOW)) {
    //     Serial.println("State change from LOW to HIGH");
    //     eventFlag = true;
    //     i++;
        // if (i == 1) {
        //     lum = MID_LUM;
        //     delayTime = 68; // for 28 secs and 5% change
        // } else if (i == 3) {
        //     lum = MID_LUM;
        //     delayTime = 24; // for 57 secs and 29% change
        // } else if (i == 5) {
        //     lum = MID_LUM;
        //     delayTime = 73; // for 30 secs and 5% change
        // } else if (i == 7) {
        //     lum = MID_LUM;
        //     delayTime = 24; // for 57 secs and 29% change
        // } else if (i == 9) {
        //     lum = MID_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 11) {
        //     lum = MID_LUM;
        //     delayTime = 33; // for 80 secs and 29% change 
        // } else if (i == 13) {
        //     lum = MID_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 15) {
        //     lum = MID_LUM;
        //     delayTime = 31; // for 75 secs and 29% change
        // } else if (i == 17) {
        //     lum = MID_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 19) {
        //     lum = MID_LUM;
        //     delayTime = 17; // for 40 secs and 29% change
        // } else if (i == 21) {
        //     lum = MID_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 23) {
        //     lum = MID_LUM;
        //     delayTime = 11; // for 27 secs and 29% change
        // } else if (i == 25) {
        //     lum = MID_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 27) {
        //     lum = MID_LUM;
        //     delayTime = 27; // for 65 secs and 29% change
        // }
    // } else if ((state == LOW) && (prevState == HIGH)) {
    //     Serial.println("State change from HIGH to LOW");
    //     eventFlag = true;
    //     i++;
        // if (i == 2) {
        //     lum = MAX_LUM;
        //     delayTime = 13; // for 31 secs and 29% change
        // } else if (i == 4) {
        //     lum = MIN_LUM;
        //     delayTime = 19;
        // } else if (i == 6) {
        //     lum = 20;
        //     delayTime = 20; // for 23 secs and 14% change
        // } else if (i == 8) {
        //     lum = MIN_LUM;
        //     delayTime = 48; //for 20 secs and 5% change
        // } else if (i == 10) {
        //     lum = MAX_LUM;
        //     delayTime = 17; //for 41 secs and 29% change
        // } else if (i == 12) {
        //     lum = MIN_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 14) {
        //     lum = MAX_LUM;
        //     delayTime = 14; // for 34 secs and 29% change
        // } else if (i == 16) {
        //     lum = MIN_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 18) {
        //     lum = MAX_LUM;
        //     delayTime = 21; // for 50 secs and 29% change
        // } else if (i == 20) {
        //     lum = MIN_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // } else if (i == 22) {
        //     lum = MAX_LUM;
        //     delayTime = 10; // for 25 secs and 29% change
        // } else if (i == 24) {
        //     lum = MIN_LUM;
        //     delayTime = 24; // for 10 secs and 5% change
        // } else if (i == 26) {
        //     lum = MAX_LUM;
        //     delayTime = 18; // for 44 secs and 29% change
        // } else if (i == 28) {
        //     lum = MIN_LUM;
        //     delayTime = 48; // for 20 secs and 5% change
        // }
    // }

    // Send data at eventFlag
    // if (eventFlag) {
    //     eventFlag = false;
    //     data[0] = lum;
    //     data[1] = delayTime;
    //     ESPNow.send_message(receiver_mac, data, 2);
    //     Serial.println("sent lum is: " + String(lum));
    //     Serial.println("sent delay is: " + String(delayTime));
    //     Serial.println("Event count: " + String(i));
    // }

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