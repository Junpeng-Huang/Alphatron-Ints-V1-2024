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
Timer surgeTimer = Timer((unsigned long)SURGE_MAX_TIME);
// Timer visTimer = Timer((unsigned long)VIS_TIMER);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
PID pid = PID(COMPASS_P, COMPASS_I, COMPASS_D, 100);
PID cameraPid = PID(CAMERA_P, CAMERA_I, CAMERA_D, 30);
PID sidewayPid = PID(SIDEWAY_P, SIDEWAY_I, SIDEWAY_D);
PID forwardPid = PID(forward_P, forward_I, forward_D);
PID centrePid = PID(centre_P, centre_I, centre_D);
PID defendPid = PID(defend_P, defend_I, defend_D, 50);

struct Move
{
	Vect moveVect;
	Vect prevVect;
	int rot;
};

int counter = 0;
int bnoCtr = 0;
bool surge = false;
Vect rest = Vect (0, 0);
Vect prevVect;
float prevballarg;

int compass_correct(float targetHeading = 0)
{
	sensors_event_t event;
	bno.getEvent(&event);
	float orient = (float)event.orientation.x;
	if (orient > 180)
	{
		orient -= 360;
	}
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}

	return pid.update(orient, targetHeading);
}

int sideways_correct(float orient = camera.ballDir, float targetHeading = 0)
{
	if (orient > 180)
	{
		orient -= 360;
	}
	orient *= -1;
	return sidewayPid.update(orient, targetHeading);
}

float centre_correct(float targetHeading = float(event.orientation.x))
{
	float orient = camera.defendGoal.arg*sin(DEG2RAD*camera.defendGoal.mag);
	// Serial.println(orient);
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}
	targetHeading = targetHeading*-1;
	return centrePid.update(orient, targetHeading);
}

int forwards_correct()
{
	float targetDist = DEFEND_DIST; //Maybe turn down the P...

	int currentDist = camera.defendGoal.mag;

	return forwardPid.update(currentDist, targetDist);
}

int defend_correct(float targetHeading = 180)
{
	float orient = camera.defendGoal.arg;

	return defendPid.update(orient, targetHeading);
}

int camera_correct(float targetHeading = 0)
{
	float orient = camera.attackGoal.arg;
	if (orient > 180)
	{
		orient = orient - 360;
	}
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}

	return cameraPid.update(orient, targetHeading);
}

Move attack(Vect ball, bool ballVis, double outDir, double outSpd, double lineAngle, double heading, bool captured){
	Move movement;
	Orbit::OrbitData orbitData = orbit.update(camera.ball);
	if (lineAngle != -1 || outAvoidance.botlocation == -1){
		if (outAvoidance.botlocation >= 0 && (floatMod(360 - ball.arg, 360) < lineAngle - 90 || floatMod(360 - ball.arg, 360) > lineAngle + 90) && ballVis){
			movement.moveVect = Vect(orbitData.ball);
		} else {
			movement.moveVect = Vect(outSpd, outDir);
		}
	} else {
		if (ballVis) {
			if (captured && camera.attackVis) {
				movement.moveVect = Vect(STRIKE_SPEED, floatMod(360 - camera.attackGoal.arg, 360));
				// if (camera.attackGoal.arg < SHOOT_ANGLE || 360 - camera.attackGoal.arg > SHOOT_ANGLE) {
				// 	kicker.shouldKick = true;
	 			// 	kicker.kickDelay.resetTime();
				// }
			} else {
				movement.moveVect = Vect(orbitData.ball);
			}
		} else {
			// if (camera.defendVis){
			// 	if (camera.defendGoal.mag < DEFEND_DIST) {
			// 		movement.moveVect = Vect(HOMING_SPEED, camera.attackVis ? camera.attackGoal.arg : 0);
			// 	} else if (camera.defendGoal.mag > DEFEND_DIST + 5){
			// 		movement.moveVect = Vect(HOMING_SPEED, floatMod(360 - camera.defendGoal.arg, 360));
			// 	} else {
			// 		movement.moveVect = rest;
			// 	}
			// } else {
			// 	movement.moveVect = Vect(HOMING_SPEED, 180);
			// }
			movement.moveVect = rest;
		}
	}
	return movement;
}

Move defend(Vect ball, bool ballVis, double outDir, double outSpd, bool defendVis, Vect goal, double lineAngle, double heading) {
	Move move;
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}

	if (ball.mag <= DEFENSE_SURGE_STRENGTH && (camera.ballDir < 10 || camera.ballDir > 350) && goal.mag < DEFEND_DIST+5){
		surge = true;
		surgeTimer.resetTime();	
	} else if (surge && goal.mag > DEFEND_DIST+15){
		surge = false;
	} else if (surgeTimer.timeHasPassedNoUpdate()){
		surge = false;
	}

	Vect g = Vect(goal.mag, floatMod(360 - (goal.arg - heading), 360));
	Vect m = g - Vect(DEFEND_DIST*cosf(DEG2RAD*(g.arg + (ball.arg > 180 ? -2 : 2))), DEFEND_DIST*sinf(DEG2RAD*(g.arg + (ball.arg > 180 ? -2 : 2))), false);
	Vect c = g - Vect(DEFEND_DIST*cosf(DEG2RAD*(g.arg)), DEFEND_DIST*sinf(DEG2RAD*(g.arg)), false);
	Vect centre = g + Vect(DEFEND_DIST, 0);

	if(goal.mag < DEFEND_DIST + 5){ //Calibration 
		if(outDir == -1 && outAvoidance.botlocation != -1){
			if(defendVis && ballVis){
				if (surge){
					move.moveVect = Vect(DEFENSE_SUGRE_SPEED, ball.arg + heading);
				} else {
					move.moveVect = Vect((ball.arg > 180 ? 360 - ball.arg : ball.arg), m.arg); //Try vector equation cosx i + sinx j
					move.prevVect = move.moveVect;
				}
				// Serial.println("Defending");
			} else {
				move.moveVect = Vect(powf(centre.mag, 1.1), centre.arg);
				// Serial.println(move.moveVect.arg);
			}
		} else {
			move.moveVect = Vect(outSpd, outDir);
		}
    } else if (ballVis && ball.arg > 90 && ball.arg < 270){
		Orbit::OrbitData orbitData = orbit.update(camera.ball);
		move.moveVect = Vect(orbitData.ball);
	} else {
		if (defendVis) {
			move.moveVect = Vect(c.mag*DEF_MULT, c.arg);
			// Serial.println("Homing");
			Serial.println(move.moveVect.mag);
		} else {
			move.moveVect = Vect(HOMING_SPEED, 180);
		}
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
	// bluetooth.init();
	if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
		Serial.println("Error connecting to bno");
		while(1);
	}
	bno.setExtCrystalUse(true);
	delay(500);
	Serial.println("Done");
}

void loop()
{
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}
	float ol = lightsensor.update();
	float orient = ((float)event.orientation.x);
	if (orient > 180){
		orient = orient -360;
	}
  	float lineAngle = (ol != -1 ? floatMod(ol-orient, 360) : -1.00);
  	oAvoidance::Movement outavoidance = outAvoidance.moveDirection(lineAngle);
  	outavoidance.direction = (outavoidance.direction != -1 ? -1* (floatMod(outavoidance.direction, 360)) + 360 : -1.00);
	camera.update(blueAttack == false, orient);
	// // bluetooth.update(camera.ballStr);

	if (ROBOT == 2) {	// -- Attacking -- 
		Move att = attack(camera.ball, camera.ballVisible, outavoidance.direction, outavoidance.speed, lineAngle, orient, camera.inCapture);
		motors.move(att.moveVect, camera.inCapture ? camera_correct() : compass_correct(), orient);
		// Serial.println(att.moveVect.arg);
	} else {	// -- Defending --
		Move def = defend(camera.ball, camera.ballVisible, outavoidance.direction, outavoidance.speed, camera.defendVis, camera.defendGoal, lineAngle, orient);
		if (lineAngle != -1 || outAvoidance.botlocation == -1){
			motors.move(Vect(outavoidance.speed, outavoidance.direction), (camera.defendGoal.mag > DEFEND_DIST + 5 ? compass_correct() : defend_correct()), orient);
		} else {
			if (def.moveVect.mag > 0 || def.moveVect.arg > 0) {
				motors.move(Vect(def.moveVect), (camera.defendGoal.mag > DEFEND_DIST + 5 ? compass_correct() : defend_correct()), orient);
				// Serial.println(camera.ball.mag);
			} else {
				motors.move(rest, defend_correct(), orient);
			}
		}
	}

	kicker.update();

	//- Testing -//

	// Serial.println(digitalRead(SUPERTEAM_PIN));
	// bno.getEvent(&event);
	// Serial.println(event.orientation.x);
	// motors.move(40, 0, compass_correct());
	// if (camera.ballStrSurgeAvg > 180){
	// Serial.println(camera.ballDist);
	// }
	// if (camera.ballStrAvg > 160){ 
	// Serial.println(camera.ballDir);
	// }
	// Serial.println(camera.ballDir);
	// if (camera.attackAngle != 0){
	// }
	// Serial.println(event.orientation.x-camera.defendAngle);
	// Serial.println(camera.defendGoal.arg); //, camera.ballStrAvg, camera.ballVisible, outavoidance.direction, outavoidance.speed, camera.defendVis, camera.defendDist, lineAngle, orient

	// lightsensor.test();
	// Serial.println(outAvoidance.botlocation);
	// Serial.println(camera.ball.arg);
	// motors.move(outavoidance.speed, outavoidance.direction,compass_correct());
	// motors.move(0, 0, compass_correct());

	//- Kicker -//

	// digitalWrite(KICKER_PIN,HIGH);
	// delay(5000);
	// digitalWrite(KICKER_PIN,LOW);
	// Serial.println("LOW");
	// delay(50);
	// digitalWrite(KICKER_PIN,HIGH);
	// delay(5000);
	// Serial.println(floatMod(camera.yellowAngle*-1, 360));
	// Serial.println(camera.ball.mag);
}