#include "defines.h"

float floatMod(float x, float m) {
    float r = fmod(x, m);
    return r<0 ? r+m : r;
}

float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleClockwise - angleCounterClockwise, 360);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise){
    return floatMod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0, 360);
}

uint8_t mod(int8_t x, int8_t m) {
    int8_t r = x % m;
    return r < 0 ? r + m : r;
}

float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float ang = angleBetween(angleCounterClockwise, angleClockwise);
    return fmin(ang,360-ang);
}

int8_t findSign(float value) {
    return value >= 0 ? 1 : -1;
}

bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck) {
	if(angleBoundCounterClockwise < angleBoundClockwise) {
		return(angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
	} else {
		return(angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
	}
}