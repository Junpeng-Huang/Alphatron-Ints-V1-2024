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
PID ballPid = PID(CAMERA_P, CAMERA_I, CAMERA_D, 60);
PID cameraPid = PID(CAMERA_P, CAMERA_I, CAMERA_D, 100);
PID defendPid = PID(DEFEND_P, DEFEND_I, DEFEND_D);
PID sidewayPid = PID(SIDEWAY_P, SIDEWAY_I, SIDEWAY_D, 120);
PID linePid = PID(LINE_P, LINE_I, LINE_D);
PID fieldPid = PID(FIELD_P, FIELD_I, FIELD_D);
PID searchPid = PID(SEARCH_P, SEARCH_I, SEARCH_D);
ESC myESC (DRIBBLER_PWM, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)

struct Move
{
	Vect moveVect;
	int rot;
};

int counter = 0;
int bnoCtr = 0;
int oESC;
int stratNum;
bool surge = false;
bool forwardsSearch = false;
bool checkFirstCondition = true;
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

int defend_correct()
{
	return defendPid.update((angleIsInside(90, 270, camera.defendGoal.arg) ? -1 : 1)*smallestAngleBetween(270, camera.defendGoal.arg), 0);
}

int camera_correct()
{
	return cameraPid.update((angleIsInside(90, 270, camera.attackGoal.arg) ? 1 : -1)*smallestAngleBetween(90, camera.attackGoal.arg), 0);
}

int ball_correct()
{
	sensors_event_t event;
	bno.getEvent(&event);
	float orient = (float)event.orientation.x;
	if (orient > 180) {
		orient -= 360;
	}
	return angleIsInside(180, 360, floatMod(camera.ball.arg - (float)event.orientation.x, 360)) ? pid.update(orient, floatMod(camera.ball.arg - (float)event.orientation.x, 360) ? 90 : -90) : ballPid.update((angleIsInside(90, 270, camera.ball.arg) ? 1 : -1)*smallestAngleBetween(90, camera.ball.arg), 0);
}

void dribble() {
	int current = analogRead(CURRENT_PIN);
	if (current > 525 || kicker.kicking || !camera.ball.exists()) {
		myESC.speed(1100);
	} else {
		myESC.speed(int(1300 + 1.5*(angleIsInside(180, 360, camera.ball.arg) ? 0 : constrain(600/camera.ball.mag + (camera.ball.arg > 90 ? 180 - camera.ball.arg : camera.ball.arg), 0, 195))));
	}
}

void localisation(float heading) {
	Vect attackGoal = Vect(camera.attackGoal.mag, (camera.attackGoal.arg - heading));
	Vect defendGoal = Vect(camera.defendGoal.mag, (camera.defendGoal.arg - heading));
    float attackGoalX = camera.attackGoal.exists() ? attackGoal.i : defendGoalX;
    float attackGoalY = camera.attackGoal.exists() ? 100 - attackGoal.j : defendGoalY;
    float attackProx = abs((FIELD_LENGTH - 9)/2 - attackGoal.i);
    float defendGoalX = camera.defendGoal.exists() ? defendGoal.i : attackGoalX;
    float defendGoalY = camera.defendGoal.exists() ? -defendGoal.j - 100 : attackGoalY;
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

Vect adjustMovementVector(Vect currentPosition, Vect moveVect, Vect line) {
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

	// if (line.exists() && line.mag > 1) {
	// 	Vect l = line/line.mag;
	// 	moveVect = moveVect - l*(moveVect.i*l.i + moveVect.j*l.j);
	// }

	moveVect = Vect(max(moveVect.mag, 50), moveVect.arg);
    return moveVect;
}

Vect gotoPos(Vect currentPosition, Vect targetPos) {
	Vect moveVect = targetPos - Vect(currentPosition.mag, floatMod(90 - currentPosition.arg, 360));
	
	return moveVect;
}


Move attack(Vect ball, bool ballVis, double heading, bool captured, Vect line){
	Move movement;
	Orbit::OrbitData orbitData = orbit.update(camera.ball);

	if (checkFirstCondition) {
		if ((camera.attackGoal.mag - camera.defendGoal.mag) < 30) {
			forwardsSearch = false;
			checkFirstCondition = false; 
		}
	} else {
		if (!angleIsInside(60, 125, floatMod(lightsensor.lineDir - heading - 90, 360)) && !angleIsInside(235, 300, floatMod(lightsensor.lineDir - heading - 90, 360)) && lightsensor.lineDir != -1) {
			forwardsSearch = true;
			checkFirstCondition = true; 
		}
	}

	switch (stratNum){
    case 0: //First instance losing ball
        if (!ballVis){ 
			if (idleTimer.timeHasPassedNoUpdate() &&  camera.defendGoal.exists() && camera.attackGoal.exists()) {
				stratNum = -1;
			} else {
				if (camera.defendVis){
					movement.moveVect = (Vect(camera.defendGoal) + Vect(DEFEND_DIST + 20, 90))*2.5;
					movement.rot = compass_correct(0);
				} else {
					movement.moveVect = Vect(HOMING_SPEED, 180);
					movement.rot = compass_correct(0);
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
			// if (angleIsInside(180, 360, floatMod(ball.arg - heading, 360)) && strategies) {
			// 	stratNum = 2;
			// } else {
				if (captured && camera.attackGoal.exists()) {
					movement.moveVect = Vect(STRIKE_SPEED, floatMod(camera.attackGoal.arg, 360));
					movement.rot = camera_correct();
					if (camera.kickCond && lightsensor.lineDir == -1) {
						kicker.shouldKick = true;
						kicker.kickDelay.resetTime();
					}
				} else {
					movement.moveVect = Vect(orbitData.ballVect);
					movement.rot = compass_correct(0);
				}
				idleTimer.resetTime();
				stratNum = 1;
			// }
		} else {
			stratNum = 0;
		}
		break;
	case 2: //Sidelines Dribbling
		if (ballVis) {
			if (strategies && camera.attackGoal.exists()) {
				if (camera.attackGoal.mag < 110 && captured) { //Replace with position 
					movement.moveVect = Vect(ORBIT_CLOSE_SPEED, 90);
					movement.rot = constrain(camera_correct(), -30, 30);
					if (camera.kickCond && angleIsInside(90 - SHOOT_ANGLE, 90 + SHOOT_ANGLE, camera.attackGoal.arg)) {
						kicker.shouldKick = true;
						kicker.kickDelay.resetTime();
						stratNum = 1;
					}
				} else if (captured) {
					Vect o = Vect(orbitData.ballVect);
					Vect l = (line.exists() ? line/line.mag : Vect());
					Vect ol = o - l*(o.i*l.i + o.j*l.j); 
					movement.moveVect = ol + Vect((ORBIT_CLOSE_SPEED + ORBIT_FAR_SPEED)/2, 90);
					movement.rot = constrain(compass_correct(robotX < 0 ? 90 : 270), -50, 50);			
				} else {
					movement.moveVect = Vect(orbitData.ballVect);
					movement.rot = constrain(compass_correct(robotX < 0 ? 90 : 270), -50, 50);
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
				movement.rot = compass_correct(0);
				if (lightsensor.lineDir == -1 && outAvoidance.botlocation != -1) {
					movement.moveVect = Vect(ORBIT_CLOSE_SPEED, angleIsInside(90, 270, camera.defendGoal.arg) ? 15 : 165);
				} else {
					if (forwardsSearch) {
						movement.moveVect = Vect(ORBIT_CLOSE_SPEED, 90) + Vect(-fieldPid.update(floatMod(camera.defendGoal.arg - 180, 360), 0), 0);
					} else if (!forwardsSearch) {
						movement.moveVect = Vect(ORBIT_CLOSE_SPEED, 270) + Vect(-fieldPid.update(floatMod(camera.defendGoal.arg - 180, 360), 0), 0);
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

Move defend(Vect ball, bool ballVis, bool defendVis, Vect goal, double defendDist, Vect line, double heading) {
	Move move;
	
	Orbit::OrbitData orbitData = orbit.update(camera.ball);
	Vect g = Vect(goal.mag, floatMod(goal.arg - heading, 360));
	Vect b = Vect(ball.mag, floatMod(ball.arg - heading, 360));
	Vect c = g - Vect(defendDist*cosf(DEG2RAD*(g.arg)), DEFEND_DIST*sinf(DEG2RAD*(g.arg)), false);
	Vect d = b - g;
	Vect m = g + (d/d.mag)*defendDist;
	Vect l = (line.exists() ? line/line.mag : Vect());
	Vect ml = m - l*(m.i*l.i + m.j*l.j);
	Vect centre = g + Vect(defendDist, 90);

	if (ball.exists() && (ball.mag <= DEFENSE_SURGE_STRENGTH) && (angleIsInside(90 - ORBIT_STRIKE_ANGLE, 90 + ORBIT_STRIKE_ANGLE, ball.arg)) && goal.mag < defendDist + 10){
		surge = true;
		surgeTimer.resetTime();	
	} else if (surge && (camera.defendGoal.mag > 75)){
		surge = false;
	} else if (surgeTimer.timeHasPassedNoUpdate()){
		surge = false;
	}

	if (goal.exists()) {
		if (surge) {
			move.moveVect = Vect(STRIKE_SPEED, orbitData.ballVect.arg);
			if (camera.inCapture && camera.attackGoal.exists()) {
				move.moveVect = Vect(STRIKE_SPEED, floatMod(450 - camera.attackGoal.arg, 360));
				if (angleIsInside(90 - SHOOT_ANGLE, 90 + SHOOT_ANGLE, camera.attackGoal.arg)) {
					kicker.shouldKick = true;
					kicker.kickDelay.resetTime();
				}
			}
		} else if (goal.mag < defendDist + 15){ //Calibration 
			if(ballVis){
				if (angleIsInside(150, 210, g.arg - b.arg) && ball.mag < 20 && defendDist + 2.5) {
                	move.moveVect = Vect(-sidewayPid.update(floatMod(b.arg - 180, 360), g.arg), 180);
				} else {
					move.moveVect = Vect(-sidewayPid.update(ml.mag, 0), ml.arg);
				}
			} else {
				move.moveVect = Vect(centre.mag*3, centre.arg);
			}
			// Serial.println("Defend");
    	} else if (ballVis && angleIsInside(180, 360, ball.arg) && angleIsInside(210, 330, g.arg)){
			move.moveVect = Vect(orbitData.ballVect);
		} else {
			move.moveVect = Vect(c.mag, c.arg) + (ballVis ? Vect(-sidewayPid.update(ml.mag, 0), ml.arg) : Vect(centre.mag*3, centre.arg));
		} 
		// Serial.println("Homing");
	} else {
		move.moveVect = Vect(HOMING_SPEED, 180);		
	}

	return move;
}

void setup()
{
	// Serial.begin(9600);
	Serial.begin(115200);
	Serial4.begin(115200);
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
	delay(500);
	pinMode(CURRENT_PIN, INPUT);
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
	bno.getEvent(&event);
	float orient = ((float)event.orientation.x);
	if (orient > 180){
		orient = orient - 360;
	}
	lightsensor.update(floatMod(camera.defendGoal.arg - orient, 360));
	camera.update(blueAttack == false);
	if(Serial4.available()) {
    	Serial.print(Serial4.read());
  	}
	localisation(orient);
	velocity();
	dribble();
	bluetooth.update(camera.ball.i, camera.ball.j, robot.i, robot.j);
	float defendDist = DEFEND_DIST + abs(270 - (camera.defendGoal.arg - orient))/8;
	float lineAngle = lightsensor.lineDir != -1 ? floatMod(lightsensor.lineDir - orient, 360) : -1;
	oAvoidance::Movement outavoidance =  outAvoidance.moveDirection(lineAngle, lightsensor.lineWidth);
	Vect lineVect = Vect(max(-linePid.update(outavoidance.speed, 0), 0), outavoidance.direction); //((camera.defendGoal.mag - abs(180 - (camera.defendGoal.arg - orient))/6) < DEFEND_DIST_0 && camera.defendGoal.exists()) ? outAvoidance.moveDirection(-1, 0) :

	if (ROBOT == 2) {	// -- Attacking -- // bluetooth.thisData.role == ATTACK_MODE
		Move att = attack(camera.ball, camera.ballVisible, orient, camera.inCapture, lineVect);
		motors.move(att.moveVect + lineVect, att.rot, lineVect == Vect() ? 0 : orient);
		// Serial.println(att.moveVect.mag);
	} else {	// -- Defending -- //
		Move def = defend(camera.ball, camera.ballVisible, camera.defendVis, camera.defendGoal, defendDist, lineVect, orient);
		motors.move(def.moveVect + lineVect, surge ? camera_correct() : defend_correct(), surge ? 0 : orient);
	} 
	// kicker.update();

	////--- Testing ---////
	// Serial.println(lightsensor.lineWidth);
	// Serial.println(outavoidance.speed);
	// Serial.println(camera.defendGoal.arg - orient);
	// Vect line = Vect(lightsensor.lineDir == -1 && outAvoidance.botlocation != -1 ? 0 : lineWidthPid.update(outavoidance.speed, 1), floatMod(360 - (camera.defendGoal.arg - orient), 360)) + lineVect;
	// lightsensor.debug();
	// Serial.print(stratNum);
	// Serial.print("\t");
	// Serial.println(camera));
	// // motors.move(def.moveVect, defend_correct(), orient);
	// // Serial.print("\t");
}