#ifndef ORBIT_H
#define ORBIT_H

#include "defines.h"
#include "pins.h"
#include "Camera.h"
#include "vect.h"
#include "PID.h"
#include <Arduino.h>

class Orbit {
    public:
        struct OrbitData{
            Vect ball;
        };
        Orbit() {};
        OrbitData update(Vect ball);
    private:
        Vect b;
        Vect offset;
};

#endif