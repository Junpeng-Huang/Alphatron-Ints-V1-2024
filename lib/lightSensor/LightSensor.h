#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

#include "defines.h"
#include "pins.h"
#include "vect.h"
#include <Arduino.h>

class LightSensor {
    public:
        LightSensor() {}
        void init();
        void update();
        void debug();
        int getClusterNum();
        bool broke;
        Vect line = Vect();
        bool onLine = false;
    private:
        void read();
        uint8_t control[LS_CONTROL_NUM] = {MPLS4, MPLS3, MPLS2, MPLS1};
        uint8_t mux[LS_MUX_NUM] = {MPOUT2, MPOUT};
        uint16_t value[LS_NUM];
        uint16_t threshold[LS_NUM];
        uint8_t index[LS_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8, 24, 25, 26, 27, 28, 29, 30, 31, 23, 22, 21, 20, 19, 18, 17, 16};
        bool white[LS_NUM] = {0};
        Vect vector[LS_NUM] = {Vect()};
        float ls_x[LS_NUM] = {LS_X_0, LS_X_1,LS_X_2,LS_X_3,LS_X_4,LS_X_5,LS_X_6,LS_X_7,LS_X_8,LS_X_9,LS_X_10,LS_X_11,LS_X_12,LS_X_13,LS_X_14,LS_X_15,LS_X_16,LS_X_17,LS_X_18,LS_X_19,LS_X_20,LS_X_21,LS_X_22,LS_X_23,LS_X_24,LS_X_25,LS_X_26,LS_X_27,LS_X_28,LS_X_29,LS_X_30,LS_X_31};
        float ls_y[LS_NUM] = {LS_Y_0, LS_Y_1,LS_Y_2,LS_Y_3,LS_Y_4,LS_Y_5,LS_Y_6,LS_Y_7,LS_Y_8,LS_Y_9,LS_Y_10,LS_Y_11,LS_Y_12,LS_Y_13,LS_Y_14,LS_Y_15,LS_Y_16,LS_Y_17,LS_Y_18,LS_Y_19,LS_Y_20,LS_Y_21,LS_Y_22,LS_Y_23,LS_Y_24,LS_Y_25,LS_Y_26,LS_Y_27,LS_Y_28,LS_Y_29,LS_Y_30,LS_Y_31};
};

#endif