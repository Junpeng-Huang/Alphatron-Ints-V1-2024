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
        Movement moveDirection(double lineAngle, double lineWidth);
        bool overLine = false;
        int botlocation;
    private:
        double original_lineArg;
        double original_lineWidth;
        double previous_lineArg;
        double previous_lineWidth;
};

#endif