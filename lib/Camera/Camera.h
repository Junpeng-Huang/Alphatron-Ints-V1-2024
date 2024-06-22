#ifndef CAMERA_H
#define CAMERA_H

#include <defines.h>
#include "Arduino.h"
#include "vect.h"

class Camera {
public:
    Camera();
    void init();
    void update(bool attackBlue, float heading);
    float calculateAngleAddition();
    Vect attackGoal;
    Vect defendGoal;
    int yellowAngle;
    int yellowDist;
    int blueAngle;
    int blueDist;
    float ballDir;
    float ballDirCW;
    float ballStr;
    bool defendVis;
    bool attackVis;
    bool ballVisible;
    bool inCapture;
    float ballDist;
    Vect ball;

private:
    float ballPixeltoCM(float dist);
    float goalPixeltoCM(float dist);
    float prevAngY;
    float prevDistY;
    float prevAngB;
    float prevDistB;
    float prevAngBall;
    float prevDistBall;
    Vect blueGoal;
    Vect prevBlue;
    Vect yellowGoal;
    Vect prevYellow;
    Vect prevBall;
    unsigned long lastTime;
};

#endif