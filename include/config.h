#pragma once
#include <Arduino.h>

// ================= Pins =================
constexpr int SDA_PIN = 17;
constexpr int SCL_PIN = 16;

// Motor driver 
constexpr int AIN1 = 37;
constexpr int AIN2 = 36;
constexpr int BIN1 = 40;
constexpr int BIN2 = 39;
constexpr int STBY = 38;

// Encoder 
constexpr int ENC_A = 21;
constexpr int ENC_B = 47;
constexpr int ENC_C = 1;
constexpr int ENC_D = 2;

// PWM Configuration
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 8;

const int PWM_CH_L1 = 0; // AIN1
const int PWM_CH_L2 = 1; // AIN2
const int PWM_CH_R1 = 2; // BIN1
const int PWM_CH_R2 = 3; // BIN2

const int XSHUT_PIN_FRONT = 7; // Front TOF sensor
const int NEOPIXEL_PIN = 38; // Onboard NeoPixel LED

// Empirically determined constants: 

const float COUNTS_PER_CM = 3.3f; 
const float CM_PER_COUNT = 0.295f; //this was 0.33056 but changed to 0.2667
const int COUNTS_PER_CELL = 60; //
const int SPR = 34; // counts per wheel revolution

const float WHEEL_DIAMETER_CM = 3.3f;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.1416f;

