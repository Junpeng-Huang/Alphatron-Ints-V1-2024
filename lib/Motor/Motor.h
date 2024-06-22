#ifndef MOTOR_H
#define MOTOR_H

#include "pins.h"
#include "defines.h"
#include "Arduino.h"

class Motor {
	public:
		Motor(){};
		Motor(int pwm, int ina, int inb, bool rev);
		void move(int speed);
		void brake();
	private:
		int pwmPin, inaPin, inbPin;
		bool reversed;	
};

#endif