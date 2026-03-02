#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "PID.h"

class RobotServer {
public:
    RobotServer(PIDController<float>* turn, PIDController<float>* pos, 
                PIDController<float>* rVel, PIDController<float>* lVel);

    // Added callbacks for the movement functions
    void begin(const char* ssid, const char* password, 
               void (*restartFunc)(), 
               void (*posFunc)(int), 
               void (*turnFunc)(float));

private:
    AsyncWebServer _server;
    PIDController<float>* _turn;
    PIDController<float>* _pos;
    PIDController<float>* _rVel;
    PIDController<float>* _lVel;
    
    void (*_restartCallback)();
    void (*_posCallback)(int);
    void (*_turnCallback)(float);
    
    String getParamHTML(String name, PIDController<float>* pid, String id);
};

#endif