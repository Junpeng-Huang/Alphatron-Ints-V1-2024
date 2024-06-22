#include "kicker.h"

void Kicker::init()
{
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    kickCharge.resetTime();
    kickDischarge.resetTime();
    kickDelay.resetTime();
}

void Kicker::update()
{
    if (kicked) {
        if (this->kickDischarge.timeHasPassedNoUpdate())
        {
            // if(attackDist < 60){
            digitalWrite(KICKER_PIN, HIGH);
            this->kickCharge.resetTime();
            kicked = false;
            Serial.println("not kickedddddddddddddddddddddddddddddddddddddd");
        }
    } else {
        if (this->kickCharge.timeHasPassedNoUpdate() && shouldKick){
            if (this->kickDelay.timeHasPassedNoUpdate()){
                digitalWrite(KICKER_PIN, LOW);
                this->kickDischarge.resetTime();
                kicked = true;
                shouldKick = false;
                Serial.println("kickeddddddddddddddddddddddddddddddd");
            }
        } else {
            shouldKick = false;
            // Serial.println("Waiting");
        }
    }
}
