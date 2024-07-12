#ifndef CAMERA_H
#define CAMERA_H

#include <defines.h>
#include "Arduino.h"
#include "vect.h"

class Camera {
public:
    Camera();
    void init();
    void update(bool attackBlue);
    float calculateAngleAddition();
    Vect attackGoal;
    Vect defendGoal;
    float ballDir;
    bool defendVis;
    bool attackVis;
    bool ballVisible;
    bool inCapture;
    bool kickCond;
    bool cameraLock;
    float ballDist;
    Vect ball;

private:
    Vect ballPixeltoCM(Vect ball);
    Vect goalPixeltoCM(Vect goal);
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