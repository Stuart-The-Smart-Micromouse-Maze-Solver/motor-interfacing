#include "status_led.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void statusLedInit() {
  pixel.begin();
  pixel.setBrightness(64); // 25% brightness (255 * 0.25 = 63.75)
  pixel.clear();
  pixel.show();
}

void statusLedGreen() {
  pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // Green
  pixel.show();
}

void statusLedRed() {
  pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // Red
  pixel.show();
}

void statusLedOff() {
  pixel.clear();
  pixel.show();
}
