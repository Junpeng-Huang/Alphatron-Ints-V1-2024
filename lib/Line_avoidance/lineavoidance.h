#ifndef LINEAVOIDANCE_H
#define LINEAVOIDANCE_H
#include "defines.h"
#include "pins.h"
#include "LightSensor.h"
#include <Arduino.h>
class oAvoidance {
    public:
        struct Movement{
            double speed;
            double direction;
        };
        Movement moveDirection(double lineAngle);
        bool overLine = false;
        int botlocation;
    private:
        double original_line;
        double previous_line;
};

#endif