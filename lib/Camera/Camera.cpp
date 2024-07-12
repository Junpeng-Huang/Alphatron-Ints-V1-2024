#include "Camera.h"
#include "Timer.h"
#include "vect.h"

Timer yellowTimer = Timer((unsigned long)VIS_TIMER);
Timer blueTimer = Timer((unsigned long)VIS_TIMER);
Timer ballTimer = Timer((unsigned long)VIS_TIMER);

Camera::Camera() {}

void Camera::init() {
    Serial7.begin(CAMERA_BAUD);
}

void Camera::update(bool attackBlue) {
    if(Serial7.available() >= 8) {
        uint8_t firstByte = Serial7.read();
        uint8_t secondByte = Serial7.peek();
        // Serial.println("Reading");
        if(firstByte == 255 && secondByte == 255) {
            Serial7.read();
            int yellowY = Serial7.read();
            if (yellowY != 120) {
                yellowY *= 1.05;
            }
            int yellowX = Serial7.read();
            if (yellowX != 120 || yellowY != 120) {
                attackBlue ? defendVis = true : attackVis = true;
                yellowGoal = goalPixeltoCM(Vect(120-yellowX, 120-yellowY, false));
                prevYellow = yellowGoal;
                yellowTimer.resetTime();
            } else {
                if (yellowTimer.timeHasPassedNoUpdate()){
                    attackBlue ? defendVis = false : attackVis = false;

                    yellowGoal = Vect(0, 0);
                } else {
                    yellowGoal = prevYellow;
                    attackBlue ? defendVis = true : attackVis = true;
                }
            }

            int blueY = Serial7.read();
            if (blueY != 120) {
                blueY *= 1.05;
            }
            int blueX = Serial7.read();
            if (blueX != 120 || blueY != 120) {
                attackBlue ? attackVis = true : defendVis = true;
                blueGoal = goalPixeltoCM(Vect(120-blueX, 120-blueY, false));
                prevBlue = blueGoal;
                blueTimer.resetTime();
            } else {
                if (blueTimer.timeHasPassedNoUpdate()){
                    attackBlue ? attackVis = false : defendVis = false;
                    blueGoal = Vect(0, 0);
                } else {
                    blueGoal = prevBlue;
                    attackBlue ? attackVis = true : defendVis = true;
                }
            } 

            int ballY = Serial7.read();
            int ballX = Serial7.read();
            if (ballX != 120 || ballY != 120) {
                ballVisible = true;
                ball = ballPixeltoCM(Vect(120-ballX, 120-ballY, false));
                prevBall = ball;
                ballTimer.resetTime();
            } else {
                if (ballTimer.timeHasPassedNoUpdate()){
                    ballVisible = false;
                    ball = Vect(0, 0);
                } else {
                    ball = prevBall;
                    ballVisible = true;
                }
            }
            
            attackGoal = (attackBlue ? blueGoal : yellowGoal);
            defendGoal = (attackBlue ? yellowGoal : blueGoal);
            inCapture = (angleIsInside(90 - ORBIT_STRIKE_ANGLE, 90 + ORBIT_STRIKE_ANGLE, ball.arg) && ball.mag < CAPTURE_DIST && ballVisible ? true : false);
            kickCond = (inCapture && angleIsInside(90 - SHOOT_ANGLE, 90 + SHOOT_ANGLE, attackGoal.arg) ? true : false);
            cameraLock = (angleIsInside(30, 150, ball.arg) && ball.mag < DIST_BEHIND_BALL*2 && ballVisible ? true : false);
        }
    }
}

Vect Camera::ballPixeltoCM(Vect ball){
    return (ball.exists()) ? Vect(1.51439 * expf(0.0405471 * ball.mag) + 1.43081, floatMod(-ball.arg, 360)) : Vect();
}

Vect Camera::goalPixeltoCM(Vect goal){
    return (goal.exists()) ? Vect(0.0129052 * expf(0.0494145 * (goal.mag + 80.9429)) + 1.3936, floatMod(-goal.arg, 360)) : Vect();
}