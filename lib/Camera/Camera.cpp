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

void Camera::update(bool attackBlue, float heading) {
    if(Serial7.available() >= 8) {
        uint8_t firstByte = Serial7.read();
        uint8_t secondByte = Serial7.peek();
        // Serial.println("Reading");
        if(firstByte == 255 && secondByte == 255) {
            Serial7.read();

            int yellowX = Serial7.read();
            if (yellowX != 120) {
                yellowX *= 1.05;
            }
            int yellowY = Serial7.read();
            yellowAngle = (RAD2DEG * atan2(240-yellowY-120, 240-yellowX-120)) + 180;
            if(yellowAngle < 0){
                yellowAngle += 360;
            }
            yellowDist = sqrt(((240-yellowX-120)*(240-yellowX-120))+((240-yellowY-120)*(240-yellowY-120)));
            if (yellowDist != 0) {
                attackBlue ? defendVis = true : attackVis = true;
                yellowGoal = Vect(yellowDist, yellowAngle);
                prevYellow = Vect(yellowDist, yellowAngle);
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

            int blueX = Serial7.read();
            if (blueX != 120) {
                blueX *= 1.05;
            }
            int blueY = Serial7.read();
            blueAngle = (RAD2DEG * atan2(240-blueY-120, 240-blueX-120)) + 180; 
            if(blueAngle < 0){
                blueAngle += 360;
            }
            blueDist = sqrt(((240-blueX-120)*(240-blueX-120))+((240-blueY-120)*(240-blueY-120)));
            if (blueDist != 0) {
                attackBlue ? attackVis = true : defendVis = true;
                blueGoal = Vect(blueDist, blueAngle);
                prevBlue = Vect(blueDist, blueAngle);
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

            int ballX = Serial7.read();
            int ballY = Serial7.read();
            ballDir = RAD2DEG * atan2(240-ballY-120, 240-ballX-120) + 180;
            if(ballDir > 0){
                ballDir -= 360;
            }
            ballDir *= -1;
            ballDist = ballPixeltoCM(sqrt(((240-ballX-120)*(240-ballX-120))+((240-ballY-120)*(240-ballY-120))));
            if (ballDist != 0.00) {
                ballVisible = true;
                ball = Vect(ballDist, ballDir);
                prevBall = Vect(ballDist, ballDir);
                ballTimer.resetTime();
                // Serial.println("Visible");
            } else {
                if (ballTimer.timeHasPassedNoUpdate()){
                    ballVisible = false;
                    ball = Vect(0, 0);
                    // Serial.println("Not Visible");
                } else {
                    ball = prevBall;
                    ballVisible = true;
                    // Serial.println("Delay Tracking");
                }
            }

            attackGoal = (attackBlue ? blueGoal : yellowGoal);
            defendGoal = (attackBlue ? yellowGoal : blueGoal);
            inCapture = ((ball.arg > 360-ORBIT_STRIKE_ANGLE || ball.arg < ORBIT_STRIKE_ANGLE) && ball.mag < CAPTURE_DIST && ballVisible ? true : false);
        }
    }
}

float Camera::ballPixeltoCM(float dist){
    return (dist != 0) ? 1.51439 * expf(0.0405471 * dist) + 1.43081 : 0;
}

float Camera::goalPixeltoCM(float dist){
    return (dist != 0) ? 5.7478f * powf(10.0f, -13.0f) * expf(0.0494379*(dist + 552.825f)) + 13.8327f : 0;
}