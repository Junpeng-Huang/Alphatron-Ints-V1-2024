#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Timer.h>
#include <defines.h>
#include <pins.h>
#include "SoftwareSerial.h"
#include "vect.h"

class Bluetooth
{
    private:
        void send();
        void receive();
        void decideRole();
        Timer sendTimer = Timer(BT_SEND_TIMER);
        Timer connectedTimer = Timer(CONNECTED_TIMER);
        uint16_t sameRole = 0;
        
    public:
        void init();
        void update(float ballX, float ballY, float robotX, float robotY); 
        BluetoothData thisData = {0, -1};
        BluetoothData otherData = {0, -1};
        Timer switchTimer = Timer(BT_SWITCH_TIMER);
        bool isConnected = false;
        bool isSwitching = false;
        Vect ball;
        Vect robot;
        Vect thisBall;
        Vect otherBall;
        Vect thisRobot;
        Vect otherRobot;
        int defendDist;      

};


#endif