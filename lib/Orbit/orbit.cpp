#include "orbit.h"
#include "vect.h"

Orbit::OrbitData Orbit::update(Vect ball){
    OrbitData output;
    Camera camera;
    if (ball.arg != -1){
        b = Vect(ball.mag, ball.arg);
        if(ball.mag > DIST_BEHIND_BALL + 15) {  
            output.ball = Vect(STRIKE_SPEED, ball.arg);
        } else if (ball.arg > 270 || ball.arg < 90) {
            ball = b - Vect(DIST_BEHIND_BALL + (1.7292*expf(0.0304233*(ball.arg < 90 ? ball.arg : 360 - ball.arg)) - 26.7292 + BALL_OFFSET), 0);
            double angleDiff = (ball.arg > 180 ? 0.1*abs(ball.arg-360) + ball.mag : 0.1*ball.arg + ball.mag);
            double cVal = ORBIT_CLOSE_SPEED/1.2;
            ball.mag = constrain((angleDiff) + cVal, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ball = Vect((b.arg > 360-ORBIT_STRIKE_ANGLE || b.arg < ORBIT_STRIKE_ANGLE) ? STRIKE_SPEED : ball.mag, ball.arg);
            // Serial.print("  ");
            // Serial.println(output.ball.arg);
        } else {
            double angleDiff = (ball.arg > 180 ? 0.1*abs(ball.arg-360) + ball.mag : 0.1*ball.arg + ball.mag);
            double cVal = ORBIT_CLOSE_SPEED/1.2;
            ball.mag = constrain((angleDiff) + cVal, ORBIT_CLOSE_SPEED, ORBIT_FAR_SPEED);
            output.ball = Vect(ball.mag, ball.arg < 180 ? ball.arg + 90 : ball.arg - 90);
            // Serial.print(output.ball.mag);
            // Serial.print("  ");
            // Serial.println(output.ball.arg);
        }
    }
    return output;
}