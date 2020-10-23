#include "PID.h"

PID::PID(double P, double I, double D, double SP){
    
    Kp = P;
    Ki = I;
    Kd = D;
    
    SampleTime = 0.00;
    
    SetPoint = SP;
    
	maxREF = 0.00;
	minREF = 0.00;

    clear();
}

PID::PID(double P, double I, double D, double SP, double ct){
    
    Kp = P;
    Ki = I;
    Kd = D;
    
    SampleTime = 0.00;

    current_time = ct;
    last_time = current_time;
    
    SetPoint = SP;
    
	maxREF = 0.00;
	minREF = 0.00;
	
    clear();
}

void PID::clear(){
    
    /*clears PID computations and coefficients*/
    
    PTerm = 0.00;
    ITerm = 0.00;
    DTerm = 0.00;
    last_error = 0.00;
    
    /*Windup Guard*/
    windup_guard = GUARD;
    
    cnt = 0;

    output = 0.00;
    
}

double PID::update(double feedback_value){
    /*Calculate error with calibration*/
    if(minREF || maxREF != 0.00)
		error = SetPoint - (MAX_VAL * (feedback_value - minREF) / (maxREF - minREF));
    else
		error = SetPoint - feedback_value;
    
    /*calculate PID*/
    if (error > LEVEL|| error < -LEVEL){
        delta_error = error - last_error;
        
        PTerm = Kp * error;
        ITerm += error;
            
        if(ITerm < -windup_guard)
            ITerm = -windup_guard;
        else if(ITerm > windup_guard)
            ITerm = windup_guard;
       
        DTerm = 0.00;
        DTerm = delta_error;

        last_error = error;
            
        output = PTerm + (Ki * ITerm) + (Kd * DTerm);
    }    
    else{ ITerm = 0.00; return 0;}
    
    return output;
}

double PID::update(double feedback_value, double ct){
    /*Calculate error with calibration*/
	if(minREF || maxREF != 0.00)
		error = SetPoint - (MAX_VAL * (feedback_value - minREF) / (maxREF - minREF));
    else
		error = SetPoint - feedback_value;
	
    current_time = ct;
    delta_time =  current_time - last_time;
    delta_error = error - last_error;
    if (error > LEVEL|| error < -LEVEL){
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
            last_time = current_time;
        
            last_error = error;
            
            output = PTerm + (Ki * ITerm) + (Kd * DTerm);
            
        }
    }
    else{
        cnt++;
        //syslog(LOG_NOTICE, "%d", cnt);
        tslp_tsk(2000);
        if(cnt > 5) {ITerm = 0.00; cnt = 0; return 0;}
        return error;
    }

    return output;
}

void PID::setSampleTime(double st){
    SampleTime = st;
}

void PID::setRefMinMax(double min, double max){
	minREF = min;
    maxREF = max;
}