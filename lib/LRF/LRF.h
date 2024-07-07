#ifndef LRF_H
#define LRF_H

#include "defines.h"
#include "vect.h"
#include "Arduino.h"
#include <Wire.h>
#include <VL53L1X.h>

class LRF {
	public:
		LRF() {};
        void init();
		void update(float heading);
        Vect robot = Vect();
        int bestCntX = 0;
        int bestCntY = 0;
        float bestPosX = 0;
        float bestPosY = 0;
	private:
        const uint8_t sensorCount = LRF_NUM;
        const uint8_t xshutPins[uint8_t(LRF_NUM)] = {0, 5, 8, 10, 11, 26, 40, 41};
        VL53L1X sensors[uint8_t(LRF_NUM)];
        int sensorRad = LRF_RAD;
        float fieldLen = FIELD_LENGTH;
        float fieldWid = FIELD_WIDTH;
        float thresh = LRF_THRESHOLD;
        int tryNewX = 0;
        int tryNewY = 0;
        bool triedNewX = false;
        bool triedNewY = false;
        int tryNewThresh = CHANGE_THRESH;
        float maxChange = MAX_CHANGE_THRESH;
        bool lrfLocalise[8] = {true};
        float posX[8] = {0};
        float posY[8] = {0};
        float prevposX = 0;
        float prevposY = 0;
};

#endif