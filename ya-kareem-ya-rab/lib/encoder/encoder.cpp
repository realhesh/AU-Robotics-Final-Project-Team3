#include <Arduino.h>
#include "encoder.h"

encoder::encoder(uint8_t encoder_A, uint8_t encoder_B) {
    pinMode(encoder_A, INPUT);
    pinMode(encoder_B, INPUT);
    ticks = 0;
}

unsigned long encoder::getTicks() {
    return ticks;
}

void encoder::incrementTicks() {
    ticks += ticks;
}

void encoder::resetTicks() {
    ticks = 0;
}