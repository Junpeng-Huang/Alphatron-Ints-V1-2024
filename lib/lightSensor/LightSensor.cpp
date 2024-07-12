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
    for (uint8_t i = 0; i < LS_NUM; i++) {
        vector[i] = Vect(ls_x[i], ls_y[i], false); //Check
    }
}

void LightSensor::update(float defendAngle)
{
    read();
    uint8_t clusterNum = 0;
    uint8_t lsInCluster[4] = {0};
    bool inCluster = false;
    Vect cluster[4];
    TSection = false;

    for (uint8_t i = 0; i < LS_NUM; i++) { 
        if (!inCluster) {
            if (white[i]) {
                inCluster = true;
                cluster[clusterNum] += vector[i];
                lsInCluster[clusterNum]++;
            } //Sets as start
        } else {
            if (white[i]) {
                cluster[clusterNum] += vector[i];
                lsInCluster[clusterNum]++;
            } else {
                inCluster = false;
                cluster[clusterNum] /= lsInCluster[clusterNum];
                clusterNum++;
            } //Ends & records cluster
        }
    }
    if (white[LS_NUM-1]) { //LS31 -> LS0 Case
        if(white[0]) {
            cluster[0] = (cluster[0]*lsInCluster[0] + cluster[clusterNum])/(lsInCluster[0] + lsInCluster[clusterNum]); 
        } else {
            cluster[clusterNum] /= lsInCluster[clusterNum];
            clusterNum++;
        } 
    }

    if (clusterNum == 0) {
        line = Vect(); 
    } else if (clusterNum == 3) {
        TSection = true;
        float angleDiff12 = angleBetween(cluster[0].arg, cluster[1].arg);
        float angleDiff23 = angleBetween(cluster[1].arg, cluster[2].arg);
        float angleDiff31 = angleBetween(cluster[2].arg, cluster[0].arg);
        float biggestAngle = fmax(angleDiff12, fmax(angleDiff23, angleDiff31));
        if (biggestAngle == angleDiff12) {
            line = cluster[2];
        } else if (biggestAngle == angleDiff23) {
            line = cluster[0];
        } else {
            line = cluster[1];
        }
    } else if (clusterNum == 2) {
        if (abs(lsInCluster[1] - lsInCluster[0]) > 3 && angleIsInside(170, 205, angleBetween(cluster[0].arg, cluster[1].arg)) && !angleIsInside(125, 235, defendAngle)) {
            line = (lsInCluster[1] > lsInCluster[0] ? cluster[1] : cluster[0]);
            TSection = true;
        } else {
            line = (cluster[0] + cluster[1])/2;
        }
    } else if (clusterNum == 1) {
        line = cluster[0];
    }
    lineDir = (clusterNum != 0 ? line.arg : -1);
    lineWidth = line.mag;
}

void LightSensor::debug()
{
    update(0);

    for (int i = 0; i < LS_NUM; i++)
    {
        if (value[i] > 15)
        {
            broke = false;
        }
        else
        {
            broke = true;
            break;
        }
    }

    for (int j = 0; j < LS_NUM; j++) {
        Serial.print(white[j]);
        Serial.print(" ");
    }
    Serial.print(" ");
    // Serial.print(clusterNum);
    // Serial.print("\t");
    // Serial.print(lineDir);
    // Serial.print("\t");
    // Serial.println(lineWidth);

    // Serial.println(read[19]);

    // for(int i=0; i<32; i++){
    //     Serial.print(value[i]);
    //     Serial.print(" ");
    // }
    // Serial.println(" ");
    // delay(50);
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
        if(white[mod(i+1, LS_NUM)] && white[mod(i-1, LS_NUM)]) {
            white[i] = true;
        } 
    }
}