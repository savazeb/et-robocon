#ifndef PID_H
#define PID_H

class PID{
    
    private:
    void init(double P, double I, double D, double SetPoint);
    void clear();
    double update(double feedback_value , double current_time);
    void setSampleTime(double st);
    
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

#endif
