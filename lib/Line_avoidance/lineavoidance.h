#ifndef LINEAVOIDANCE_H
#define LINEAVOIDANCE_H
#include "defines.h"
#include "pins.h"
#include "LightSensor.h"
#include <Arduino.h>
class oAvoidance {
    public:
        struct Movement{
            Vect lineAvoid;
        };
        Movement moveDirection(Vect line, bool lineVis);
        bool overLine = false;
        int botlocation;
    private:
        Vect original_line = Vect();
        Vect previous_line = Vect();
};

#endif