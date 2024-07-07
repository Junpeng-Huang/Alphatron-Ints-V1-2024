#include "orbit.h"
#include "vect.h"
#include "PID.h"

Orbit::OrbitData Orbit::update(Vect ball){
    OrbitData output;
    Camera camera;
    PID orbitPID = PID((float)((60)/(2*DIST_BEHIND_BALL)), 0, 0);

    if (ball.arg != -1){
        b = Vect(ball.mag, ball.arg);
        if (ball.arg > 270 || ball.arg < 90) {
            ball = b - Vect(max(-orbitPID.update(ball.arg < 180 ? ball.arg : 360 - ball.arg, 0), 0), 0);
            double angleDiff = (ball.arg > 180 ? 0.1*abs(ball.arg-360) + ball.mag : 0.1*ball.arg + ball.mag);
            double cVal = ORBIT_CLOSE_SPEED/1.2;
            ball.mag = constrain((angleDiff) + cVal, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ball = Vect(ball.mag, (b.arg > 360-ORBIT_STRIKE_ANGLE || b.arg < ORBIT_STRIKE_ANGLE) ? b.arg : ball.arg); //(b.arg > 360-ORBIT_STRIKE_ANGLE || b.arg < ORBIT_STRIKE_ANGLE) ? STRIKE_SPEED :
        } else {
            double angleDiff = ball.mag;
            double cVal = ORBIT_CLOSE_SPEED/1.2;
            ball = b + (ball.arg < 180 ? Vect(DIST_BEHIND_BALL, ball.arg + 90) : Vect(DIST_BEHIND_BALL, ball.arg - 90));
            ball.mag = constrain((angleDiff) + cVal, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ball = Vect(ball.mag, ball.arg);
        }
    }
    return output;
}
