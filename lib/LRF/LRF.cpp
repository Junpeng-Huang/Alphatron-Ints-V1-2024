#include "LRF.h"

void LRF::init() { //On Teensy 4.0
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
 
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
      deadSense[i] = 1;
    }
    else {
      // Each sensor must have its address changed to a unique value other than
      // the default of 0x29 (except for the last one, which could be left at
      // the default). To make it simple, we'll just count up from 0x2A.
      sensors[i].setAddress(0x2A + i);
      sensors[i].setDistanceMode(VL53L1X::DistanceMode::Medium);
      sensors[i].startContinuous(33);
    }
  }
}
 

void LRF::update(float heading) {
  ////Serial.print(heading);
  ////Serial.println("-------------------------------------------------------------------------------");
 Serial.println("-------------------------------------------------------------------------------");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    int distance = 9999999;
    if (deadSense[i] == 0) {
      distance = (sensors[i].read() + sensorRad)/10;
    }
    Serial.print(distance);
    Serial.print('\t');
    int senseDir = floatMod(360 + LRFangles[i] - heading, 360);
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
    posY[i] = frontWall - distance*sin(senseDir*0.0174533);
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
    posX[i] = sideWall - distance*cos(senseDir*0.0174533);
  }
  Serial.println();
  for (int i = 0; i < 8; i++) {
    Serial.print(posX[i]);
    Serial.print('\t');
  }
  Serial.println();
  for (int i = 0; i < 8; i++) {
    Serial.print(posY[i]);
    Serial.print('\t');
  }
  float bestPosX = 0;
  float bestUncX = 100;
  int bestCntX = 0;
 
  float bestPosY = 0;
  float bestUncY = 100;
  int bestCntY = 0;
 
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
      bestPosX = avgX;
      bestUncX = uncX;
      bestCntX = cntX;
    }
    float avgY = sumY / cntY;
    float uncY = maxY - minY;
    if ((cntY > bestCntY) || ((cntY == bestCntY) && (uncY < bestUncY))) {
      bestPosY = avgY;
      bestUncY = uncY;
      bestCntY = cntY;
    }
  }
 
  Serial.println();
  Serial.print(bestPosX); //2 bytes (multiply by 100)
  Serial.print('\t');
  Serial.print(bestCntX); //1 byte (int)
  Serial.print('\t');
  Serial.print(bestUncX);
  Serial.print('\t');
  Serial.print(bestPosY); //2 bytes (multiply by 100)
  Serial.print('\t');
  Serial.print(bestCntY); //1 byte (int)
  Serial.print('\t');
  Serial.println(bestUncY);
 
  // Send positions and counts to 4.1
  int16_t sendPosX = bestPosX * 100 + 1;
  int16_t sendPosY = bestPosY * 100 + 1;
  uint8_t sendCntX = bestCntX;
  uint8_t sendCntY = bestCntY;
  Serial1.write(byte(255));
  Serial1.write(byte(255));
  Serial1.write(byte(sendPosX>>8));
  Serial1.write(byte(sendPosX));
  Serial1.write(byte(sendPosY>>8));
  Serial1.write(byte(sendPosY));
  Serial1.write(byte(sendCntX));
  Serial1.write(byte(sendCntY));
  delay(1000);
}