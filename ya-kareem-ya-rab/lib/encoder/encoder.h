#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class encoder {
public:
    volatile unsigned long ticks;

    encoder(uint8_t encoder_A, uint8_t encoder_B);
    unsigned long getTicks();
    void incrementTicks();  // helper to increment safely
    void resetTicks();      // optional: reset counter
};

#endif // ENCODER_H