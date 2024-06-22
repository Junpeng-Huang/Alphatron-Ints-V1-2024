#include "movement.h"
#include "Adafruit_BNO055.h"

Movement::Movement(){
    bl = Motor(PWMBL, BLINA, BLINB, true);
    br = Motor(PWMBR, BRINA, BRINB, true);
    fl = Motor(PWMFL, FLINA, FLINB, true);
    fr = Motor(PWMFR, FRINA, FRINB, true);
}

void Movement::move(Vect move, int rotation, float heading){
    float a = move.mag * sinf(DEG2RAD * (move.arg - 45 - heading));
    float b = -move.mag * sinf(DEG2RAD * (move.arg - 135 - heading));
    float values[4] = {rotation + a, rotation + b, rotation - a, rotation - b};
    float maxVal = max(max(max(abs(values[0]), abs(values[1])), abs(values[2])), values[3]);
    if(maxVal > 255){
        for(int i = 0; i < MOTOR_NO; i++) {
            values[i] = (values[i]/maxVal)*255;
        }
    }
    fr.move(values[0]);
    fl.move(values[1]);
    bl.move(values[2]);
    br.move(values[3]);
}