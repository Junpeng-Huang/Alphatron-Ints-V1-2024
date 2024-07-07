#ifndef KICKER_H
#define KICKER_H

#include "defines.h"
#include "pins.h"
#include "Timer.h"
#include <Arduino.h>

class Kicker {
    public:
        void update();
        void init();
        bool kicked = false;
        bool shouldKick = false;
        bool kicking = false;
        Timer kickDelay = Timer((unsigned long)KICK_DELAY_TIME);
    private:
        bool triggered = false;
        uint8_t i = 0;
        Timer kickDischarge = Timer((unsigned long)KICK_DISCHARGE_TIME);
        Timer kickCharge = Timer((unsigned long)KICK_CHARGE_TIME);
        float lgCalVal;
};

#endif