#include "motor.h"

Motor::Motor(int pwm, int ina, int inb, bool rev){
    pwmPin = pwm;
    inaPin = ina;
    inbPin = inb;

    reversed = rev;

    pinMode(pwmPin, OUTPUT);
    pinMode(inaPin, OUTPUT);
    pinMode(inbPin, OUTPUT);
}

void Motor::move(int speed) {
    if (reversed){
        if (speed > 0) {
            analogWrite(pwmPin, constrain(speed,0,255));
            digitalWriteFast(inaPin, HIGH);
            digitalWriteFast(inbPin, LOW);
        }
        else if (speed < 0) {
            analogWrite(pwmPin, constrain(abs(speed),0,255));
            digitalWriteFast(inaPin, LOW);
            digitalWriteFast(inbPin, HIGH);
        }
        else {
            analogWrite(pwmPin, 0);
            digitalWriteFast(inaPin, LOW);
            digitalWriteFast(inbPin, LOW);
        }
    } else {
        if (speed > 0) {
            analogWrite(pwmPin, constrain(speed,0,255));
            digitalWriteFast(inaPin, LOW);
            digitalWriteFast(inbPin, HIGH);
        }
        else if (speed < 0) {
            analogWrite(pwmPin, constrain(abs(speed),0,255));
            digitalWriteFast(inaPin, HIGH);
            digitalWriteFast(inbPin, LOW);
        }
        else {
            analogWrite(pwmPin, 0);
            digitalWriteFast(inaPin, LOW);
            digitalWriteFast(inbPin, LOW);
        }
    }
}