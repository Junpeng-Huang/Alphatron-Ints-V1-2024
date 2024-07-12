#include "orbit.h"
#include "vect.h"
#include "PID.h"

Orbit::OrbitData Orbit::update(Vect ball){
    OrbitData output;
    Camera camera;
    PID orbitPID = PID(1, 0, 0);

    if (ball.arg != -1){
        b = Vect(ball.mag, ball.arg);
        if (angleIsInside(0, 180, ball.arg)) {
            ballVect = b - Vect(expf((log(DIST_BEHIND_BALL + 1)/90)*abs(orbitPID.update(b.arg, 90))), 90);
            double angleDiff = (ball.arg > 90 ? (180 - ball.arg)/90: ball.arg/90);
            ballVect.mag = constrain((ORBIT_FAR_SPEED - ORBIT_CLOSE_SPEED)*angleDiff + ORBIT_CLOSE_SPEED, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ballVect = Vect(ballVect.mag, ballVect.arg); //(b.arg > 360-ORBIT_STRIKE_ANGLE || b.arg < ORBIT_STRIKE_ANGLE) ? STRIKE_SPEED :
        } else {
            double angleDiff = ball.mag;
            double cVal = ORBIT_CLOSE_SPEED/1.2;
            ballVect = b + (ball.arg < 270 ? Vect(DIST_BEHIND_BALL, ball.arg + 90) : Vect(DIST_BEHIND_BALL, ball.arg - 90));
            ballVect.mag = constrain((angleDiff) + cVal, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ballVect = Vect(ballVect.mag, ballVect.arg);
        }
    } else {
        ballVect = Vect();
    }
    return output;
}
