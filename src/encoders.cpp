#include <Arduino.h>
#include "encoders.h"
#include "config.h"
#include "soc/gpio_struct.h"

// ESP32-S3: GPIO 0-31 → GPIO.in, GPIO 32+ → GPIO.in1.data
#define FAST_READ(p) ((p) < 32 ? ((GPIO.in >> (p)) & 1u) : ((GPIO.in1.data >> ((p) - 32)) & 1u))

// Define the globals (IMPORTANT: this allocates them)
Encoder leftEncoder  = {0, 0, 0};
Encoder rightEncoder = {0, 0, 0};

// Protect 64-bit shared variable access between ISR and main loop
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

int64_t readEncoderCounts(const Encoder& encoder)
{
  noInterrupts();
  int32_t c = encoder.counts;
  interrupts();
  return (int64_t)c;
}

void resetEncoderCounts() {
  noInterrupts();
  leftEncoder.counts = 0;
  rightEncoder.counts = 0;
  interrupts();
}

void encodersInit()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);

  // Trigger on A and C rising; use B and D to decide direction
  attachInterrupt(digitalPinToInterrupt(ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D), rightEncoderISR, CHANGE);
}

void IRAM_ATTR leftEncoderISR()
{
  static uint8_t lastState = 0;

  uint8_t A = FAST_READ(ENC_A);
  uint8_t B = FAST_READ(ENC_B);

  uint8_t state = (A << 1) | B;
  uint8_t combined = (lastState << 2) | state;

  switch (combined)
  {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      leftEncoder.counts--; // flipped left encoder
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      leftEncoder.counts++; // flipped left encoder
      break;
  }

  lastState = state;
}

void IRAM_ATTR rightEncoderISR()
{
  static uint8_t lastState = 0;

  uint8_t C = FAST_READ(ENC_C);
  uint8_t D = FAST_READ(ENC_D);

  uint8_t state = (C << 1) | D;
  uint8_t combined = (lastState << 2) | state;

  switch (combined)
  {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      rightEncoder.counts++;
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      rightEncoder.counts--;
      break;
  }

  lastState = state;
}



// #define COUNTS_PER_REV 35   // yo is this true??
#define COUNTS_PER_REV 140


void readBothEncoders(int32_t& leftOut, int32_t& rightOut) {
  noInterrupts();
  leftOut  = leftEncoder.counts;
  rightOut = rightEncoder.counts;
  interrupts();
}

float readRPM(Encoder& encoder)
{
  noInterrupts();
  int32_t c = encoder.counts;
  interrupts();

  unsigned long now = micros();
  unsigned long dt_us = now - encoder.lastReadTime;

  if (dt_us == 0) return 0.0f;

  int32_t dc = c - encoder.lastCounts;

  encoder.lastCounts = c;
  encoder.lastReadTime = now;

  float dt_min = dt_us / 60000000.0f;  // us -> minutes
  float revs = dc / (float)COUNTS_PER_REV;

  return revs / dt_min;
}

float readAvgPosition() {
  int32_t left, right;
  readBothEncoders(left, right);
  return (left + right) / 2.0f;
}