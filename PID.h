//
//  PID.h
//
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#ifndef ____PID__
#define ____PID__

struct PIDData{
    float time, target, input, output, p, i, d;
};

class PID{
public:
    PID(float kp, float ki, float kd);
    
    float compute(float target, float input, float time);
    
    float compute(PIDData& data);

    void tune(float kp, float ki, float kd);
    
    void tolerate(float tolerance);
    
private:
    
    float tolerance=0;
    
    bool firstLoop = true; //derivative can't be calculated on a single measure
    
    float lastTarget;
    
    long lastTime;
    
    float lastError; //used to compute derivative of error
    float accumulatedError;
    
    //proportional, integral and derivative coefficients
    float kp;
    float ki;
    float kd;

    
};


#endif /* defined(____PID__) */
