#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "motor.h"
#include "pins.h"
#include "defines.h"
#include "Arduino.h"
#include "vect.h"

class Movement {
    public:
        Movement();
        void move(Vect move, int rotation, float heading);
        void brake(int power) {this->move(Vect(0,0), power, 0); bl.brake(); fl.brake(); br.brake(); fr.brake();}
    private:
        Motor bl, fl, br, fr;
        float motor_angle[4] = {45, 135, 225, 315};
};

#endif