#include <Arduino.h>
#include <math.h>
#include "Movement.h"
#include "pins.h"
#include "vect.h"
#include "defines.h"
#include "Adafruit_BNO055.h"
#include "PID.h"
#include "camera.h"
#include "orbit.h"
#include "LightSensor.h"
#include "lineavoidance.h"
#include "Camera.h"
#include "time.h"
#include "kicker.h"
#include "Bluetooth.h"
#include "LRF.h"
#include "Servo.h"
#include "ESC.h"
#include <Wire.h>

using namespace bon;
sensors_event_t event;
Movement motors;
Orbit orbit;
LightSensor lightsensor;
oAvoidance outAvoidance;
Camera camera;
Kicker kicker;
Bluetooth bluetooth;
LRF lrf;
Servo servo;
Timer surgeTimer = Timer((unsigned long)SURGE_MAX_TIME);
Timer idleTimer = Timer((unsigned long)IDLE_TIMER);
Timer searchTimer = Timer((unsigned long)SEARCH_TIMER);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
PID pid = PID(COMPASS_P, COMPASS_I, COMPASS_D, 100);
PID cameraPid = PID(CAMERA_P, CAMERA_I, CAMERA_D, 100);
PID defendPid = PID(DEFEND_P, DEFEND_I, DEFEND_D);
PID sidewayPid = PID(SIDEWAY_P, SIDEWAY_I, SIDEWAY_D, ORBIT_FAR_SPEED);
PID linePid = PID(LINE_P/10, LINE_I, LINE_D);
PID lineWidthPid = PID(LINE_P/5, LINE_I, LINE_D);
PID fieldPid = PID(FIELD_P, FIELD_I, FIELD_D);
PID searchPid = PID(SEARCH_P, SEARCH_I, SEARCH_D);
ESC myESC (DRIBBLER_PWM, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)

struct Move
{
	Vect moveVect;
};

int counter = 0;
int bnoCtr = 0;
int oESC;
int stratNum;
bool surge = false;
bool forwardsSearch = true;
bool oneWaySearch = false; //One way is right
float confidenceLvl;
bool strategies = true;
float attackGoalX = 0;
float attackGoalY = 0;
float defendGoalX = 0;
float defendGoalY = 0;
float robotX = 0;
float robotY = 0;
bool inPosition = false;
const int numLoops = 50;
Vect ballPositions[numLoops];
unsigned long times[numLoops];
Vect velocityReadings[numLoops];
int currentIndex = 0;
bool initialized = false;
Vect ballVelocity;
Vect robot = Vect();

int compass_correct(float targetHeading)
{
	sensors_event_t event;
	bno.getEvent(&event);
	float orient = (float)event.orientation.x;
	if (orient > 180) {
		orient -= 360;
	}
	if (targetHeading > 180) {
		targetHeading = targetHeading - 360;
	}
	// Serial.println(targetHeading);

	return pid.update(orient, targetHeading);
}

int defend_correct(float targetHeading = 180)
{
	float orient = camera.defendGoal.arg;

	return defendPid.update(orient, targetHeading);
}

int camera_correct(float targetHeading = 0)
{
	float orient = camera.attackGoal.arg;
	if (orient > 180) {
		orient = orient - 360;
	}
	if (targetHeading > 180) {
		targetHeading = targetHeading - 360;
	}

	return cameraPid.update(orient, targetHeading);
}

int ball_correct(float targetHeading = camera.ball.arg)
{
	sensors_event_t event;
	bno.getEvent(&event);
	float orient = (float)event.orientation.x;
	if (orient > 180) {
		orient = orient - 360;
	}
	if (targetHeading > 180) {
		targetHeading = targetHeading - 360;
	}
	constrain(targetHeading, -90, 90);

	return cameraPid.update(orient, targetHeading);
}

void dribble() {
	int current = analogRead(CURRENT_PIN);
	if (current > 530 || kicker.kicking) {
		myESC.speed(1000);
	} else {
		myESC.speed(1000 + constrain(600/camera.ball.mag + (camera.ball.arg > 180 ? camera.ball.arg - 180 : 180 - camera.ball.arg)/2, 0, 175));
	}
}

void localisation(float heading) {
	Vect attackGoal = Vect(camera.attackGoal.mag, (camera.attackGoal.arg - heading));
	Vect defendGoal = Vect(camera.defendGoal.mag, (camera.defendGoal.arg - heading));
    float attackGoalX = camera.attackGoal.exists() ? attackGoal.j : defendGoalX;
    float attackGoalY = camera.attackGoal.exists() ? 100 - attackGoal.i : defendGoalY;
    float attackProx = abs((FIELD_LENGTH - 9)/2 - attackGoal.i);
    float defendGoalX = camera.defendGoal.exists() ? defendGoal.j : attackGoalX;
    float defendGoalY = camera.defendGoal.exists() ? -defendGoal.i - 100 : attackGoalY;
    float defendProx = abs((FIELD_LENGTH - 9)/2 + defendGoal.i);

    float cameraConfidence = (camera.attackGoal.exists() + camera.defendGoal.exists())/2;
    float LRFConfidence = Vect(lrf.bestPosX, lrf.bestPosY).exists() ? constrain((((1/5)*lrf.bestCntX - 1/5) + ((1/3)*lrf.bestCntY - 1/3)), 0, 2) : 0;
    // cameraConfidence + LRFConfidence > 1.5 ? strategies = true : strategies = false;
	if (cameraConfidence + LRFConfidence != 0) {
		robotX = (cameraConfidence*(attackGoalX*attackProx + defendGoalX*defendProx)/(attackProx + defendProx) + lrf.bestPosX*LRFConfidence)/(cameraConfidence + LRFConfidence);
    	robotY = (cameraConfidence*(attackGoalY*attackProx + defendGoalY*defendProx)/(attackProx + defendProx) + lrf.bestPosX*LRFConfidence)/(cameraConfidence + LRFConfidence);
	}
    robot = (cameraConfidence + LRFConfidence != 0) ? Vect(robotX, robotY, false) : Vect();
}


void velocity() {
    Vect ballCurrentPosition = camera.ball;
    unsigned long currentTime = micros();
    if (initialized) {
        int prevIndex = (currentIndex + 1)%numLoops;
        Vect previousBallPosition = ballPositions[prevIndex];
        unsigned long previousTime = times[prevIndex];
        float dt = (currentTime - previousTime);
        if (dt != 0) {
            Vect ds = ballCurrentPosition - previousBallPosition;
            Vect currentVelocity = ds*100000/dt;
            velocityReadings[currentIndex] = currentVelocity;
            Vect avgVelocity = Vect();
            for (int i = 0; i < numLoops; i++) {
                avgVelocity = avgVelocity + velocityReadings[i];
            }
            ballVelocity = avgVelocity/numLoops;
        }
    } else {
        velocityReadings[currentIndex] = Vect();
    }

    ballPositions[currentIndex] = ballCurrentPosition;
    times[currentIndex] = currentTime;
    currentIndex = (currentIndex + 1)%numLoops;
    if (currentIndex == 0 && !initialized) {
        initialized = true;
    }
}

Vect adjustMovementVector(Vect currentPosition, Vect moveVect) {
	Vect projectedPos = currentPosition + moveVect;
    if (projectedPos.i > LINE_LOC_X) {
        moveVect = moveVect*((LINE_LOC_X - currentPosition.i)/moveVect.i);
    } else if (projectedPos.i < -LINE_LOC_X) {
        moveVect = moveVect*((-LINE_LOC_X - currentPosition.i)/moveVect.i);
    }

    if (projectedPos.j > LINE_LOC_Y) {
        moveVect = moveVect*((LINE_LOC_Y - currentPosition.j)/moveVect.j);
    } else if (projectedPos.j < -LINE_LOC_Y) {
        moveVect = moveVect*((-LINE_LOC_Y - currentPosition.j)/moveVect.j);
    }
	moveVect = Vect(max(moveVect.mag, 50), moveVect.arg);
    return moveVect;
}

Vect gotoPos(Vect currentPosition, Vect targetPos) {
	Vect moveVect = targetPos - Vect(currentPosition.mag, floatMod(90 - currentPosition.arg, 360));
	
	return moveVect;
}


Move attack(Vect ball, bool ballVis, double heading, bool captured){
	Move movement;
	Orbit::OrbitData orbitData = orbit.update(camera.ball);

	if ((camera.attackGoal.mag - camera.defendGoal.mag) < 30 && forwardsSearch) {
		forwardsSearch = false;
	} else if (!angleIsInside(60, 125, lightsensor.line.arg) && !angleIsInside(235, 300, lightsensor.line.arg) && lightsensor.onLine) {
		forwardsSearch = true;
	}

	switch (stratNum){
    case 0: //First instance losing ball
        if (!ballVis){ 
			if (idleTimer.timeHasPassedNoUpdate() &&  camera.defendGoal.exists() && camera.attackGoal.exists()) {
				stratNum = -1;
			} else {
				if (camera.defendVis){
					movement.moveVect = (Vect(camera.defendGoal.mag, floatMod(360 - camera.defendGoal.arg, 360)) + Vect(DEFEND_DIST + 20, 0))*2.5;
				} else {
					movement.moveVect = Vect(HOMING_SPEED, 180);
				}
				stratNum = 0;
				searchTimer.resetTime();
			}
		} else {
			stratNum = 1;
		}
		break;
    case 1: //Orbit attack
		if (ballVis && camera.attackGoal.exists()) {
			if (angleIsInside(90, 270, floatMod(ball.arg + heading, 360)) && strategies) {
				stratNum = 2;
			} else {
				if (captured && camera.attackGoal.exists()) {
					movement.moveVect = Vect(orbitData.ball.mag, floatMod(360 - camera.attackGoal.arg, 360));
					if (camera.attackGoal.arg < 180 ? camera.attackGoal.arg < SHOOT_ANGLE : 360 - camera.attackGoal.arg < SHOOT_ANGLE && camera.kickCond) {
						kicker.shouldKick = true;
						kicker.kickDelay.resetTime();
					}
				} else {
					movement.moveVect = Vect(orbitData.ball);
				}
				idleTimer.resetTime();
				stratNum = 1;
			}
		} else {
			stratNum = 0;
		}
		break;
	case 2: //Sidelines Dribbling
		if (ballVis) {
			if (strategies && camera.attackGoal.exists()) {
				if (captured && lightsensor.onLine) {
					if (camera.attackGoal.arg < 180 ? camera.attackGoal.arg < SHOOT_ANGLE : 360 - camera.attackGoal.arg < SHOOT_ANGLE && camera.attackGoal.mag < 50) {
						kicker.shouldKick = true;
						kicker.kickDelay.resetTime();
					} else {
						movement.moveVect = Vect(orbitData.ball) + Vect(ORBIT_FAR_SPEED, 0);
					}
				} else {
					movement.moveVect = Vect(orbitData.ball);
				}
				idleTimer.resetTime();
			} else {
				stratNum = 1;
			}
		} else {
			stratNum = 0;
		}
		break;
	case -1: //Search algorithm
		if (ballVis) {
			stratNum = 1;
		} else {
			if (searchTimer.timeHasPassedNoUpdate()){
				idleTimer.resetTime();
				stratNum = 0;
			} else {
				if (!lightsensor.onLine && outAvoidance.botlocation != -1) {
					movement.moveVect = Vect(oneWaySearch ? max(-fieldPid.update(camera.defendGoal.arg < 180 ? camera.defendGoal.arg : camera.defendGoal.arg - 360, 0), 0) : -fieldPid.update(camera.defendGoal.arg < 180 ? camera.defendGoal.arg : camera.defendGoal.arg - 360, 0), 90);
				} else {
					if (forwardsSearch) {
						movement.moveVect = Vect(ORBIT_FAR_SPEED, 0) + Vect(-fieldPid.update(camera.defendGoal.arg < 180 ? camera.defendGoal.arg : camera.defendGoal.arg - 360, 0), 90);
					} else if (!forwardsSearch) {
						movement.moveVect = Vect(ORBIT_FAR_SPEED, 180) + Vect(-fieldPid.update(camera.defendGoal.arg < 180 ? camera.defendGoal.arg : camera.defendGoal.arg - 360, 0), 90);
					} else {
						movement.moveVect = Vect();
					}
				}
				stratNum = -1;
			}
		}
		break;
    default: 
        stratNum = 0;
        break;
    }	
	return movement;
}

Move defend(Vect ball, bool ballVis, bool defendVis, Vect goal, double defendDist, double lineWidth, double heading) {
	Move move;
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}

	if (ball.exists() && ball.mag <= DEFENSE_SURGE_STRENGTH && (ball.arg < ORBIT_STRIKE_ANGLE*2 || ball.arg > 360 - ORBIT_STRIKE_ANGLE*2) && goal.mag < defendDist + 5){
		surge = true;
		surgeTimer.resetTime();	
	} else if (surge && (camera.attackGoal.mag < camera.defendGoal.mag)){
		surge = false;
	} else if (surgeTimer.timeHasPassedNoUpdate()){
		surge = false;
	}
	
	Orbit::OrbitData orbitData = orbit.update(camera.ball);
	Vect g = Vect(goal.mag, floatMod(360 - (goal.arg - heading), 360));
	Vect b = Vect(ball.mag, floatMod(ball.arg + heading - 180, 360));
	Vect c = g - Vect(defendDist*cosf(DEG2RAD*(g.arg)), DEFEND_DIST*sinf(DEG2RAD*(g.arg)), false);
	float swdc = constrain(c.mag*2, 15, 30);
	// Vect interceptionPoint = calculateInterceptionPoint(g, b, ballVelocity, defendDist);
	Vect m = g - Vect(defendDist*cosf(DEG2RAD*(g.arg + (ball.arg > 180 ? -swdc*2 : swdc*2))), DEFEND_DIST*sinf(DEG2RAD*(g.arg + (ball.arg > 180 ? -swdc : swdc))), false);
	Vect centre = g + Vect(defendDist, 0);

	if (goal.exists()) {
		if (surge) {
			move.moveVect = Vect(STRIKE_SPEED, orbitData.ball.arg);
			if (camera.inCapture && (camera.attackGoal.arg < 180 ? camera.attackGoal.arg < SHOOT_ANGLE : 360 - camera.attackGoal.arg < SHOOT_ANGLE) && camera.attackGoal.exists()) {
				kicker.shouldKick = true;
				kicker.kickDelay.resetTime();
			}
		} else if ((lightsensor.onLine || outAvoidance.botlocation == -1) && goal.mag < defendDist + 5){ //Calibration 
			if(ballVis){
				if (angleIsInside(150, 210, ball.arg) && ball.mag < 20 && defendDist + 2.5) {
                	move.moveVect = Vect((ball.arg > 180 ? ball.arg - 180 : 180 - ball.arg), m.arg) + (!lightsensor.onLine && outAvoidance.botlocation != -1 ? Vect() : Vect(lineWidthPid.update(lineWidth, 1), g.arg));
				} else {
					move.moveVect = Vect((ball.arg > 180 ? 1 : -1)*sidewayPid.update(b.arg, g.arg), m.arg) + (!lightsensor.onLine && outAvoidance.botlocation != -1 ? Vect() : Vect(lineWidthPid.update(lineWidth, 1), g.arg));
				}
			} else {
				move.moveVect = Vect(centre.mag*3, centre.arg) + (lightsensor.onLine ? Vect(lineWidthPid.update(lineWidth, 1), g.arg) : Vect());
			}
    	} else if (ballVis && angleIsInside(90, 270, ball.arg) && angleIsInside(120, 240, g.arg)){
			move.moveVect = Vect(orbitData.ball);
		} else {
			move.moveVect = Vect(c.mag*4, c.arg);
		}
	} else {
		move.moveVect = Vect(HOMING_SPEED, 180);		
	}
	return move;
}

void setup()
{
	Serial.begin(9600);
	camera.init();
	delay(500);
 	lightsensor.init();
	Wire.begin();
	bno.begin();
	kicker.init();
	bluetooth.init();
	// lrf.init();
	if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
		Serial.println("Error connecting to bno");
		while(1);
	}
	bno.setExtCrystalUse(true);
	// delay(500);
	// pinMode(CURRENT_PIN, INPUT);
	// myESC.arm();
	// delay(7000);
	// for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  
	// 	myESC.speed(oESC);                                    
	// 	delay(10);
	// } for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  
    // 	myESC.speed(oESC);                                    
    // 	delay(10);                                            
   	// }                                         
	Serial.println("Done");
}

void loop()
{
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}
	float orient = ((float)event.orientation.x);
	if (orient > 180){
		orient = orient - 360;
	}
	lightsensor.update();
	// 	lrf.update(orient);
	camera.update(blueAttack == false);
	localisation(orient);
	velocity();
	bluetooth.update(camera.ball.i, camera.ball.j, robot.i, robot.j);
	float defendDist = DEFEND_DIST + abs(180 - (camera.defendGoal.arg - orient))/4;
	Vect line = (lightsensor.onLine ? Vect(lightsensor.line.mag, floatMod(lightsensor.line.arg - orient - 90, 360)) : Vect());
  	oAvoidance::Movement outavoidance = outAvoidance.moveDirection(line, lightsensor.onLine);
	Vect lineVect = ((lightsensor.onLine || outAvoidance.botlocation == -1) ? Vect(max(-linePid.update(outavoidance.lineAvoid.mag, 0), 0), -1*(floatMod(outavoidance.lineAvoid.arg, 360)) + 360) : Vect());
	// dribble();

	if (ROBOT == 2) {	// -- Attacking -- // bluetooth.thisData.role == ATTACK_MODE
		Move att = attack(camera.ball, camera.ballVisible, orient, camera.inCapture);
		motors.move(adjustMovementVector(Vect(robot), att.moveVect) + lineVect, (camera.inCapture && camera.attackGoal.mag < 50) ? camera_correct() : compass_correct(floatMod(camera.ball.arg + orient, 360) > 180 ? 270 : 90), lineVect == Vect() ? 0 : orient);
	} else {	// -- Defending -- //
		Move def = defend(camera.ball, camera.ballVisible, camera.defendVis, camera.defendGoal, defendDist, outavoidance.lineAvoid.mag, orient);
		motors.move(def.moveVect + lineVect, camera.inCapture ? camera_correct() : defend_correct(), orient);

	} 
	// kicker.update(); 

	////--- Testing ---////
	// Serial.println(camera.ball.arg);
	// Serial.println(outavoidance.speed);
	// Serial.println(camera.defendGoal.mag);
	// Vect line = Vect(lightsensor.lineDir == -1 && outAvoidance.botlocation != -1 ? 0 : lineWidthPid.update(outavoidance.speed, 1), floatMod(360 - (camera.defendGoal.arg - orient), 360)) + lineVect;
	// lightsensor.debug();
	// Serial.print("\t");
	// Serial.print(line.mag);
	// Serial.print("\t");
	// Serial.println(lightsensor.lineDir);
	// motors.move(def.moveVect, defend_correct(), orient);
	// Serial.print("\t");
	// Serial.println(line.arg);
	// Serial.print(def.moveVect.mag);
	// Serial.print("\t");
	// Serial.println(def.moveVect.arg);
}