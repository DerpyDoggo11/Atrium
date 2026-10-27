#pragma once
#include <Arduino.h>

// ESP32
static constexpr uint8_t PIN_ESP_TX = 0;
static constexpr uint8_t PIN_ESP_RX = 1;
static constexpr uint8_t PIN_ESP_EN = 2;
static constexpr uint8_t PIN_ESP_BOOT = 3;

// GNSS
static constexpr uint8_t PIN_GNSS_TX = 4; 
static constexpr uint8_t PIN_GNSS_RX = 5; 
static constexpr uint8_t PIN_GNSS_NRESET = 6;
static constexpr uint8_t PIN_GNSS_WAKEUP = 7;
static constexpr uint8_t PIN_GNSS_PPS = 8;
static constexpr uint8_t PIN_GNSS_ANTOFF = 9;

static constexpr uint32_t GNSS_BAUD = 9600; 

// Status
static constexpr uint8_t PIN_LED_A = 10;
static constexpr uint8_t PIN_LED_B = 11; 

#define STATUS_LED_ACTIVE_LOW 1

static constexpr uint8_t PIN_BUZZER = 25;

// Breakout 
static constexpr uint8_t PIN_ESC[4] = {20, 21, 22, 23};
static constexpr uint8_t PIN_J5_SPARE = 24;

static constexpr uint8_t PIN_J6[5] = {12, 13, 14, 15, 19};

static constexpr uint8_t PIN_J4_ADC[3] = {27, 28, 29};

// Power 
static constexpr uint8_t PIN_VBAT_ADC = 26; 
static constexpr float VBAT_DIVIDER = 2.0f;
static constexpr float ADC_VREF = 3.3f;