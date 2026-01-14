#include "config.h"
#include "encoders.h"

Encoder leftEncoder  = {0, 0, 0};
Encoder rightEncoder = {0, 0, 0};

// Shared mux for safe 64-bit access
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

int64_t readEncoderCounts(const Encoder& e)
{
  portENTER_CRITICAL(&encMux);
  int64_t c = e.counts;
  portEXIT_CRITICAL(&encMux);
  return c;
}

void resetEncoderCounts()
{
  portENTER_CRITICAL(&encMux);
  leftEncoder.counts = 0;
  rightEncoder.counts = 0;
  portEXIT_CRITICAL(&encMux);
}

// Quadrature ISRs (same logic you already had)
static void IRAM_ATTR leftEncoderISR_A()
{
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&encMux);
  leftEncoder.counts += (a == b) ? -1 : 1;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void IRAM_ATTR leftEncoderISR_B()
{
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&encMux);
  leftEncoder.counts += (a != b) ? -1 : 1;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void IRAM_ATTR rightEncoderISR_C()
{
  int c = digitalRead(ENC_C);
  int d = digitalRead(ENC_D);

  portENTER_CRITICAL_ISR(&encMux);
  rightEncoder.counts += (c == d) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void IRAM_ATTR rightEncoderISR_D()
{
  int c = digitalRead(ENC_C);
  int d = digitalRead(ENC_D);

  portENTER_CRITICAL_ISR(&encMux);
  rightEncoder.counts += (c != d) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}

void encodersInit()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), leftEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), leftEncoderISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C), rightEncoderISR_C, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D), rightEncoderISR_D, CHANGE);

  leftEncoder.prevTime  = micros();
  rightEncoder.prevTime = micros();
  leftEncoder.prevCounts  = readEncoderCounts(leftEncoder);
  rightEncoder.prevCounts = readEncoderCounts(rightEncoder);
}
