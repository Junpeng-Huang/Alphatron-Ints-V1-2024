#include "lineavoidance.h"
LightSensor sensor;

oAvoidance::Movement oAvoidance::moveDirection(Vect line, bool lineVis){
    
    oAvoidance::Movement movement;
    switch (botlocation){
    case 0:
        if (lineVis){ //sees line
            if (original_line == Vect()){ //sees line for first time
                original_line = Vect(1 - line.mag, line.arg);
            }
            botlocation = 1; //sets location of bot on the line
        }
        else{
            ///nothinga
            movement.lineAvoid = Vect();
            original_line = Vect();
            botlocation = 0;
            break;
        }
    case 1:
        if (!lineVis){ //If line no longer seen: back inside field of play
            movement.lineAvoid = Vect();
            original_line = Vect();
            botlocation = 0;
            // Serial.println("Error");
        }
        else if (smallestAngleBetween(line.arg, original_line.arg) >= 120){ //If new line is on opposite side of robot: Outside field of play
            movement.lineAvoid = Vect(line.mag + 1, line.arg);
            botlocation = -1;
        }
        else{ //Inside field of play: On line
            movement.lineAvoid = Vect(1-line.mag, floatMod((line.arg-180), 360));
            botlocation = 1;
            // Serial.print(lineAngle);
        }
        break;
    case -1:
        // Serial.println(smallestAngleBetween(lineAngle, original_line));
        if (!lineVis){ //Doesn't see line when outside field of play: Fully outside
            movement.lineAvoid = Vect(3, previous_line.arg);
            botlocation = -1;
            // Serial.println("Full Out");
        }
        else if (smallestAngleBetween(line.arg, original_line.arg) < 80){ //Moved back inside field of play
            movement.lineAvoid = Vect(1 - line.mag, line.arg);
            botlocation = 1;
            // Serial.println("Back in #1");
        }
        else {//Outside field of play: on line
            movement.lineAvoid = Vect(line.mag + 1, line.arg);
            botlocation = -1;
            // Serial.println("Out on line");
        }
        break;
    default:
        ////unkown state /////
        botlocation = 0;
        break;
    }
    previous_line = lineVis ? line : previous_line;

    return movement;
}