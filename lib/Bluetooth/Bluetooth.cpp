#include "Bluetooth.h"

void Bluetooth::init() {
    Serial5.begin(BLUETOOTH_BAUD);
}

void Bluetooth::update(float ballX, float ballY, float robotX, float robotY){ 
    thisData.ballX = (uint8_t)ballX;
    thisData.ballY = (uint8_t)ballY;
    thisData.robotX = (uint8_t)robotX;
    thisData.robotY = (uint8_t)robotY;
    if(sendTimer.timeHasPassed()) {
        send();
    }
    receive();
    decideRole();

    isConnected = connectedTimer.timeHasPassedNoUpdate() ? false : true;

    if (isSwitching){
        Serial.println("Switching");
    }
    // if (isConnected){
    //     Serial.println("Connected");
    // }

    // Serial.println(thisData.role);
}

void Bluetooth::send(){
    Serial5.write(BT_START_BYTE);
    Serial5.write(BT_START_BYTE);
    Serial5.write(thisData.ballX);
    Serial5.write(thisData.ballY);
    Serial5.write(thisData.robotX);
    Serial5.write(thisData.robotY);
    Serial5.write(thisData.role);
}

void Bluetooth::receive(){
    uint8_t buffer[BT_PACKET_SIZE - 2] = {0};
    if(Serial5.available() >= BT_PACKET_SIZE) {
        uint8_t firstByte = Serial5.read();
        uint8_t secondByte = Serial5.peek();
        if(firstByte == BT_START_BYTE && secondByte == BT_START_BYTE) {
            Serial5.read();
            for(uint8_t i = 0; i < BT_PACKET_SIZE - 2; i++) {
                buffer[i] = Serial5.read();
            }
            connectedTimer.resetTime();
            otherData.ballX = buffer[0];
            otherData.ballY = buffer[1];
            otherData.robotX = buffer[2];
            otherData.robotY = buffer[3];
            bool roleBefore = otherData.role;
            otherData.role = buffer[4] == DEFENSE_MODE ? DEFENSE_MODE : ATTACK_MODE;
            if(!roleBefore && otherData.role){
                if(otherData.role == thisData.role){
                    isSwitching = true;
                }
            } else if(otherData.role == thisData.role){
                isSwitching = true;
            }
        }
    }
}

void Bluetooth::decideRole() {
    thisBall = Vect(thisData.ballX, thisData.ballY, false);
    otherBall = Vect(otherData.ballX, otherData.ballY, false);
    thisRobot = Vect(thisData.robotX, thisData.robotY, false);
    otherRobot = Vect(otherData.robotX, otherData.robotY, false);

    if(thisData.role == DEFENSE_MODE){
        if(thisBall.mag < DEFENSE_SURGE_STRENGTH){
            thisData.role = ATTACK_MODE;
        } else if(otherBall.mag - thisBall.mag >= 40) {
            thisData.role = ATTACK_MODE;
        }
        if ((thisData.role == otherData.role)){
            if((thisBall.mag < otherBall.mag) && thisBall.exists()){
                thisData.role = ATTACK_MODE;
            } else{
                thisData.role = DEFENSE_MODE;
            }
        }
    }
    if (!isConnected){
        thisData.role = DEFENSE_MODE;
        otherData.role = -1;
        otherBall = Vect();
        sameRole = 0;
    } else if (isSwitching){
        thisData.role = !thisData.role;
        isSwitching = false;
        switchTimer.resetTime();
        sameRole = 0;
    }
}

// uint8_t firstByte = Serial5.read();
// if (firstByte != -1) {
//     Serial.println(firstByte);
// }
// Serial.println(Serial5.available());
// while (Serial5.available()){
//     Serial.println(Serial5.read());
// }