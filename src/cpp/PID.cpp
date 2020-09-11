#include <time.h>

#include "PID.h"

PID::PID(double P, double I, double D, double SP, double ct){
    
    Kp = P;
    Ki = I;
    Kd = D;
    
    SampleTime = 0.00;

    current_time = ct;
    last_time = current_time;
    
    SetPoint = SP;
    
    clear();
}

void PID::clear(){
    
    /*clears PID computations and coefficients*/
    
    PTerm = 0.00;
    ITerm = 0.00;
    DTerm = 0.00;
    last_error = 0.00;
    
    /*Windup Guard*/
    int_error = 0.00;
    windup_guard = 22;
    
    output = 0.00;
    
}

double PID::update(double feedback_value, double ct){
    /*Calculate error with calibration*/
    error = SetPoint - (100 * (feedback_value - 2) / (33 - 2));
    
    current_time = ct;
    delta_time =  current_time - last_time;
    delta_error = error - last_error;
    
    if (delta_time >= SampleTime){
        PTerm = Kp * error;
        ITerm += error * delta_time;
        
        if(ITerm < -windup_guard)
            ITerm = -windup_guard;
        else if(ITerm > windup_guard)
            ITerm = windup_guard;
        
        DTerm = 0.00;
        if(delta_time > 0)
            DTerm = delta_error / delta_time;
        
        /*Remember last time and last error for next calculation*/
        if(current_time = delta_time) last_time = 0.0;
        else last_time = current_time;
        

        last_error = error;
        
        output = PTerm + (Ki * ITerm) + (Kd * DTerm);
        
    }
    
    return output;
}

void PID::setSampleTime(double st){
    SampleTime = st;
}
