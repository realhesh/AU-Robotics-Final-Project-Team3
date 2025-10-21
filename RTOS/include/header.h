#ifndef HEADER_H
#define HEADER_H

// pins
#define left_encoder_A 12
#define left_encoder_B 13
#define right_encoder_A 14
#define right_encoder_B 15

#include <servo.h>

Servo gripper;
gripper.attach(); // hnhot hna elpin elhnwsl 3leh elservo
// joystick readings
int leftx = 0, lefty = 0;
int rightx = 0, righty = 0;
int gripper = 0;

class encoder
{
private:
    unsigned long ticks;

public:
    encoder(uint8_t encoder_A, uint8_t encoder_B);

    unsigned long getTicks();
};

#endif // HEADER_H