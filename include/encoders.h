#pragma once
#include <Arduino.h>

struct Encoder {
  volatile int64_t counts;
  int64_t prevCounts;
  uint32_t prevTime;
};

extern Encoder leftEncoder;
extern Encoder rightEncoder;

void encodersInit();
int64_t readEncoderCounts(const Encoder& e);
void resetEncoderCounts();
