#include "Bluetooth.h"

void Bluetooth::init() {
    Serial5.begin(BLUETOOTH_BAUD);
}

void Bluetooth::update(float ballStrSurge){ 
    thisData.ballStrSurge = (uint8_t)ballStrSurge;
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
    Serial5.write(thisData.ballStrSurge);
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
            connectedTimer.update();
            otherData.ballStrSurge = buffer[0];
            bool roleBefore = otherData.role;
            otherData.role = buffer[1] == DEFENSE_MODE ? DEFENSE_MODE : ATTACK_MODE;
            if(!roleBefore && otherData.role){
                if(otherData.role == thisData.role){
                    isSwitching = true;
                }
            } else if(otherData.role == thisData.role){
                isSwitching = true;
            }
            // Serial.print(otherData.ballStrSurge);
            // Serial.print("  ");
            // Serial.print(otherData.role);
            // Serial.print("  ");
            // Serial.print(thisData.ballStrSurge);
            // Serial.print("  ");
            // Serial.print(thisData.role);
            // Serial.println("  ");
            // Serial.println(thisData.ballStrSurge-otherData.ballStrSurge);
        }
    }
}

void Bluetooth::decideRole() {
    if(thisData.role == DEFENSE_MODE){
        if(thisData.ballStrSurge > DEFENSE_SURGE_STRENGTH){
            thisData.role = ATTACK_MODE;
        // } else if(thisData.ballStrSurge-otherData.ballStrSurge >= 100) {
        //     thisData.role = ATTACK_MODE;
        }
        if ((thisData.role == otherData.role)){
            if(thisData.ballStrSurge > otherData.ballStrSurge){
                // Serial.print("Checking this");
                thisData.role = ATTACK_MODE;
            } else{
                thisData.role = DEFENSE_MODE;
            }
        }
    }
    if (!isConnected){
        thisData.role = DEFENSE_MODE;
        otherData.role = -1;
        otherData.ballStrSurge = 0;
        sameRole = 0;
    } else if (isSwitching){
        thisData.role = !thisData.role;
        isSwitching = false;
        switchTimer.update();
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