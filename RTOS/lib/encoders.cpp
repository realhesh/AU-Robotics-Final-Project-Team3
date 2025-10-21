#include <Arduino.h>
#include "header.h"

class encoder
{
    public:
    unsigned long ticks = 0;
    encoder(uint8_t encoder_A, uint8_t encoder_B)
    {
        pinMode(encoder_A, INPUT);
        pinMode(encoder_B, INPUT);
    }

    unsigned long getTicks(){return ticks;}
};

