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

#define CONTROL_INPUT_PIN 2
#define MAX_LUM 30
#define MIN_LUM 0

// uint8_t receiver_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
// uint8_t receiver_mac[] = {0x40, 0x22, 0xD8, 0x5E, 0x6D, 0x04};
uint8_t receiver_mac[] = {0x08, 0xB6, 0x1F, 0xB9, 0x2B, 0xE4};

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

    pinMode(CONTROL_INPUT_PIN, INPUT);
    Serial.println("Control pin number is: " + String(CONTROL_INPUT_PIN));
}

int dir = 0;
bool state = LOW;
bool prevState = LOW;
static uint8_t lum = 0;

void loop() {
    state = digitalRead(CONTROL_INPUT_PIN); // read state from control pin
    // Set direction when state changes
    if ((state == HIGH) && (prevState == LOW)) {
        dir = 1;
        Serial.println("State change from LOW to HIGH");
    } else if ((state == LOW) && (prevState == HIGH)) {
        dir = -1;
        Serial.println("State change from HIGH to LOW");
    }
    // Increase or decrease lum value
    lum += dir;
    // Stop when lum is out of bounds
    if ((lum > MAX_LUM) || (lum <= MIN_LUM)) {
        dir = 0;
    // Send lum as it changes
    } else {
        ESPNow.send_message(receiver_mac, &lum, 1);
        Serial.println(lum);
    }
    // Update previous state
    prevState = state;

    // Delay determines fade speed
    delay(100);
}