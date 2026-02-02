#include <Arduino.h>
#include "encoders.h"
#include "config.h"

// Define the globals (IMPORTANT: this allocates them)
Encoder leftEncoder  = {0};
Encoder rightEncoder = {0};

// Protect 64-bit shared variable access between ISR and main loop
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

int64_t readEncoderCounts(const Encoder& encoder)
{
  noInterrupts();
  int32_t c = encoder.counts;
  interrupts();
  return (int64_t)c;
}

void encodersInit()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);

  // Trigger on A and C rising; use B and D to decide direction
  attachInterrupt(digitalPinToInterrupt(ENC_A), leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_C), rightEncoderISR, RISING);
}

void IRAM_ATTR leftEncoderISR()
{
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&encMux);
  if (A == B) leftEncoder.counts--;
  else        leftEncoder.counts++;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR rightEncoderISR()
{
  bool C = digitalRead(ENC_C);
  bool D = digitalRead(ENC_D);

  portENTER_CRITICAL_ISR(&encMux);
  // Goal: when robot drives forward, BOTH left & right counts increase.
  if (C == D) rightEncoder.counts++;
  else        rightEncoder.counts--;
  portEXIT_CRITICAL_ISR(&encMux);

  // If forward makes right counts DECREASE, flip the ++/-- above.
}
