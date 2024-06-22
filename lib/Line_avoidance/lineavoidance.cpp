#include "lineavoidance.h"
LightSensor sensor;

oAvoidance::Movement oAvoidance::moveDirection(double lineAngle){
    oAvoidance::Movement movement;
    switch (botlocation){
    case 0:
        if (lineAngle != -1){ //sees line
            if (original_line == -1){ //sees line for first time
                original_line = lineAngle; //records original line
            }
            botlocation = 1; //sets location of bot on the line
        }
        else{
            ///nothing
            movement.direction = -1;
            movement.speed = -1;
            original_line = -1;
            botlocation = 0;
            break;
        }
    case 1:
        if (lineAngle == -1){ //If line no longer seen: back inside field of play
            movement.direction = -1;
            movement.speed = -1;
            original_line = -1;
            botlocation = 0;
            // Serial.println("Error");
        }
        else if (smallestAngleBetween(lineAngle, original_line) >= 120){ //If new line is on opposite side of robot: Outside field of play
            // if (sensor.clusterNum == 2){
            //     movement.direction -= 45;
            // }
            movement.direction = lineAngle;
            movement.speed = LS_AVOID_MEDIUM;
            botlocation = -1;
        }
        else{ //Inside field of play: On line
            movement.direction = floatMod((lineAngle-180), 360);
            movement.speed = LS_AVOID_MEDIUM;
            botlocation = 1;
            // Serial.print(lineAngle);
        }
        break;
    case -1:
        // Serial.println(smallestAngleBetween(lineAngle, original_line));
        if (lineAngle == -1){ //Doesn't see line when outside field of play: Fully outside
            movement.direction = previous_line;
            movement.speed = LS_AVOID_FAST;
            botlocation = -1;
            // Serial.println("Full Out");
        }
        else if (smallestAngleBetween(lineAngle, original_line) < 80){ //Moved back inside field of play
            movement.direction = lineAngle;
            movement.speed = LS_AVOID_MEDIUM;
            botlocation = 1;
            // Serial.println("Back in #1");
        }
        else {//Outside field of play: on line
            movement.direction = lineAngle;
            movement.speed = LS_AVOID_FAST;
            botlocation = -1;
            // Serial.println("Out on line");
        }
        break;
    default:
        ////unkown state /////
        botlocation = 0;
        break;
    }
    previous_line = (lineAngle != -1 ? lineAngle : previous_line);
    return movement;
}