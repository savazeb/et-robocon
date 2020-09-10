#pragma once
class PID{
    
    public:
    PID(double, double, double, double, double);
    double update(double, double);
    void clear();
    void setSampleTime(double);
    
    private:
    double Kp;
    double Ki;
    double Kd;
    double current_time;
    double SampleTime;
    double last_time;
    double SetPoint;
    double PTerm;
    double ITerm;
    double DTerm;
    double int_error;
    double windup_guard;
    double output;
    double error;
    double delta_time;
    double delta_error;
    double last_error;
    
};

#include "PID.cpp"
#include <ctime>
