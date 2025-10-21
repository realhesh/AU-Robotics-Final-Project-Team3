#include "header.h"


class controller()
{

    controller()
    {
        Serial.begin(115200);
        PS4.begin(""); // ESP32 bluetooth mac
        Serial.println("waiting for ps4 controller to connect");
    }

    void update()
    {
        if (PS4.isConnected())
        {
            /*
            up -127
            down +127
            left -127
            right +127
            */
            // motion
            leftx = PS4.LStickX();
            lefty = PS4.LStickY();
            leftspeed = lefty + leftx; // lsa msh 3rfa mfroud a3ml range elspeeds mn kam lkam
            rightspeed = lefty - leftx;
            // gripper lsa msh 3rfa
            rightx = PS4.RStickX();
            righty = PS4.RStickY();
        }
    }
    }

