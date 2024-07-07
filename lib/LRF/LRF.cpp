#include "LRF.h"

void LRF::init() {
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);
    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      lrfLocalise[i] = false;
    }
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);
    sensors[i].setDistanceMode(VL53L1X::DistanceMode::Short);
    sensors[i].startContinuous(100);
  }
}

void LRF::update(float heading) {
  ////Serial.print(heading);
  ////Serial.println("-------------------------------------------------------------------------------");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    for (uint8_t j = 0; j < sensorCount; j++) {
      if(!lrfLocalise[i]) {
        posX[i] = posX[j] = 9999999;
      }
    }
    int distance = (sensors[i].read() + sensorRad)/10;
    //Serial.print(distance);
    //Serial.print('\t');
    int senseDir = floatMod((450 - i*45 - heading), 360);
    //Serial.print(senseDir);
    //Serial.print('\t');
    int frontWall = 9999999;
    if (((senseDir < 8) || (senseDir > 352)) || ((senseDir > 172) && (senseDir < 188))) {
      frontWall = 9999999;
    }
    else if ((senseDir > 0) && (senseDir < 180)) {
      frontWall = fieldLen/2;
    }
    else {
      frontWall = fieldLen/-2;
    }
    posY[i] = frontWall - distance*sinf(senseDir*DEG_TO_RAD);
    int sideWall = 9999999;
    if (((senseDir > 84) && (senseDir < 96)) || ((senseDir > 264) && (senseDir < 276))) {
      sideWall = 9999999;
    }
    else if ((senseDir > 90) && (senseDir < 270)) {
      sideWall = fieldWid/-2;
    }
    else {
      sideWall = fieldWid/2;
    }
    posX[i] = sideWall - distance*cosf(senseDir*DEG_TO_RAD);
  }
  //Serial.println();
  for (int i = 0; i < 8; i++) {
    //Serial.print(posX[i]);
    //Serial.print('\t');
  }
  //Serial.println();
  for (int i = 0; i < 8; i++) {
    //Serial.print(posY[i]);
    // Serial.print('\t');
  }
  float bestUncX = 100;
  float bestUncY = 100;
  for (uint8_t i = 0; i < sensorCount; i++) {
    float sumX = 0;
    float maxX = -1000;
    float minX = 1000;
    int cntX = 0;
    float sumY = 0;
    float maxY = -1000;
    float minY = 1000;
    int cntY = 0;
    for (uint8_t j = 0; j < sensorCount; j++) {
      if ((abs(posX[i] - posX[j]) < thresh) && (abs(posX[i]) < 100) && (abs(posX[j]) < 100)) {
        cntX += 1;
        sumX += posX[j];
        if (posX[j] > maxX) {
          maxX = posX[j];
        }
        if (posX[j] < minX) {
          minX = posX[j];
        }
      }
      if ((abs(posY[i] - posY[j]) < thresh) && (abs(posY[i]) < 140) && (abs(posY[j]) < 140)) {
        cntY += 1;
        sumY += posY[j];
        if (posY[j] > maxY) {
          maxY = posY[j];
        }
        if (posY[j] < minY) {
          minY = posY[j];
        }
      }
    }
    float avgX = sumX / cntX;
    float uncX = maxX - minX;
    if ((cntX > bestCntX) || ((cntX == bestCntX) && (uncX < bestUncX))) {
        if (abs(prevposX - avgX) < maxChange || tryNewX > tryNewThresh) {
            bestPosX = avgX;
            bestUncX = uncX;
            bestCntX = cntX;
            if (tryNewX > 0) {
              tryNewX -= 1;
            }
        } else {
            tryNewX += 1;
        }
    }
    float avgY = sumY / cntY;
    float uncY = maxY - minY;
    if ((cntY > bestCntY) || ((cntY == bestCntY) && (uncY < bestUncY))) {
      if (abs(prevposY - avgY) < maxChange || tryNewY > tryNewThresh) {
            bestPosY = avgY;
            bestUncY = uncY;
            bestCntY = cntY;
            if (tryNewY > 0) {
              tryNewY -= 1;
            }
        } else {
            tryNewY += 1;
        }
    }
  }
  prevposX = bestPosX;
  prevposY = bestPosY;

  //Serial.println();
  //Serial.print(bestPosX);
  //Serial.print('\t');
  //Serial.print(bestCntX);
  //Serial.print('\t');
  //Serial.print(tryNewX);
  //Serial.print('\t');
  //Serial.print(bestPosY);
  //Serial.print('\t');
  //Serial.print(bestCntY);
  //Serial.print('\t');
  //Serial.println(tryNewY);
}