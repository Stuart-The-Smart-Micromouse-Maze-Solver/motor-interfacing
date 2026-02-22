#pragma once
#include <Arduino.h>
#include "config.h"

struct Encoder {
  volatile int32_t counts;
  volatile unsigned long lastCounts;
  volatile unsigned long lastReadTime;
};

extern Encoder leftEncoder;
extern Encoder rightEncoder;

void encodersInit();

int64_t readEncoderCounts(const Encoder& encoder);

// These MUST be declared so main.cpp can attachInterrupt() to them
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

float readRPM(Encoder& encoder);