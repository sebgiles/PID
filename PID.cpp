//
//  NXTServo.cpp
//  
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#include "PID.h"

PID::PID(float kp, float ki, float kd):
kp(kp),
ki(ki),
kd(kd)
{
}

float PID::compute(float target, float input, float time){
    PIDData data;
    
    data.target=target;
    data.input=input;
    data.time=time;
    
    return compute(data);
}

float PID::compute(PIDData& data){
    
    if (data.target!=lastTarget){ //when target changes everything is reset
        firstLoop=true;
        lastTime=data.time;
        lastError=0;
        accumulatedError=0;
    }
    
    float error = data.target - data.input;
    float dError = error - lastError;
    lastError = error;

    if (error>=-tolerance && error<=tolerance)
        error=0;
    
    float dTime = data.time - lastTime;
    lastTime = data.time;

    data.p=kp * error;
    
    if (firstLoop) {
        data.i=0;
        data.d=0;
        data.output=data.p;
        
        firstLoop=false;
    } else {
    
        //if(dError==0 && error==0) errorIntegral=0;
        
        accumulatedError+= (dTime* error);
        
        data.i = ki * accumulatedError;
        data.d = kd * dError/dTime;
        
        data.output = data.p + data.i + data.d;
    }
    return data.output;
}

void PID::tune(float newkp, float newki, float newkd){
    kp=newkp;
    ki=newki;
    kd=newkd;
}

void PID::tolerate(float newtolerance){
    tolerance=newtolerance;
}


