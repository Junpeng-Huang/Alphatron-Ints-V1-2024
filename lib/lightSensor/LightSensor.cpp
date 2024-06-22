#include "LightSensor.h"

void LightSensor::init() 
{
    for (uint8_t i = 0; i < LS_CONTROL_NUM; i++) {
        pinMode(control[i], OUTPUT);
    }
    for (uint8_t i = 0; i < LS_MUX_NUM; i++) {
        pinMode(mux[i], INPUT);
    }
    for (uint8_t i = 0; i < LS_NUM; i++)
    {
        read();
        uint16_t calibrateTotal = 0;
        for (uint8_t j = 0; j < LS_CALIBRATE_COUNT; j++)
        {
            calibrateTotal += value[i];
            delay(1);
        }
        threshold[i] = (uint16_t)(calibrateTotal /(LS_CALIBRATE_COUNT)) + LINE_BUFFER;
    }
}

void LightSensor::update()
{
    read();
    uint8_t clusterNum = 0;
    bool inCluster = false;
    uint8_t start[4] = {0};
    uint8_t end[4] = {0};

    for (uint8_t i = 0; i < LS_NUM; i++) { // reads sensors
        if (!inCluster) {
            if (white[i]) {
                inCluster = true;
                start[clusterNum] = i;
            }
        } else {
            if (!white[i]) {
                inCluster = false;
                end[clusterNum] = i;
                clusterNum++;
            }
        }
    }
    if (white[LS_NUM-1]) {
        if(white[0]) {
            start[0] = start[clusterNum];
        } else {
            end[clusterNum] = LS_NUM-1;
            clusterNum++;
        }
    }

    Vect cluster[4];
    for (uint8_t i; i < clusterNum; i++) {
        // calculater the vector of individual clusters
    }



    switch (clusterNum) {
    case 0:
        line = Vect();
        break;
    case 1:
        line = cluster[1];
        break;
    case 2:
        if (angleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint) <= 180) {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint) - LS_OFFSET, 360);
        } else {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[0].midpoint, clusterArray[1].midpoint) - LS_OFFSET, 360);
        }
        break;
    case 3:
        float angleDiff12 = angleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint);
        float angleDiff23 = angleBetween(clusterArray[2].midpoint, clusterArray[1].midpoint);
        float angleDiff31 = angleBetween(clusterArray[0].midpoint, clusterArray[2].midpoint);
        float biggestAngle = fmax(angleDiff12, fmax(angleDiff23, angleDiff31));
        if (biggestAngle == angleDiff12) {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[0].midpoint, clusterArray[1].midpoint) + LS_OFFSET, 360);
        } else if (biggestAngle == angleDiff23) {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[1].midpoint, clusterArray[2].midpoint) + LS_OFFSET, 360);
        } else {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[2].midpoint, clusterArray[0].midpoint) + LS_OFFSET, 360);
        }
        break;
    }

    return lineAngle;
}

void LightSensor::debug()
{
    read();
    for (int i = 0; i < LS_NUM; i++)
    {
        if (read[i] > 15)
        {
            broke = false;
        }
        else
        {
            broke = true;
            break;
        }
    }

    // for (int j = 0; j < LS_NUM; j++) {
    //     if (value[j] >= ls_cal[j]) {
    //         white[j] = 1;
    //     } else {
    //         white[j] = 0;
    //     }
    //     Serial.print(white[j]);
    //     Serial.print(" ");
    // }
    // Serial.println(" ");

    // Serial.println(read[19]);

    for(int i=0; i<32; i++){
        Serial.print(value[i]);
        Serial.print(" ");
    }
    Serial.println(" ");
    delay(50);
}

void LightSensor::read()
{
    for (uint8_t i = 0; i < LS_NUM_IND; i++)
    {
        for (uint8_t j = 0; j < LS_CONTROL_NUM; j++) {
            digitalWrite(control[j], (i >> j) & 0x01);
        }
        delayMicroseconds(10);
        for (uint8_t j = 0; j < LS_MUX_NUM; j++) {
            value[index[i+16*j]] = analogRead(mux[j]);
            white[index[i+16*j]] = value[index[i+16*j]] >= threshold[index[i+16*j]];
        }
    }
    for (uint8_t i = 0; i < LS_NUM; i++) {
        if(white[mod(i+1, LS_NUM)] == white[mod(i-1, LS_NUM)]) {
            white[i] = white[mod(i+1, LS_NUM)]; //Change to just white possibly
        } 
    }
}