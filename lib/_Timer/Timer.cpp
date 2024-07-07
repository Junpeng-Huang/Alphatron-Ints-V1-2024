#include "Timer.h"

Timer::Timer(unsigned long duration) {
    timerDuration = duration;
}

void Timer::resetTime() {
    lastUpdate = micros();
}


bool Timer::timeHasPassed() {
    if (micros() - lastUpdate > timerDuration) {
        resetTime();
        return true;
    }

    return false;
}

bool Timer::timeHasPassedNoUpdate() {
    return micros() - lastUpdate > timerDuration;
}
