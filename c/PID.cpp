#include <ctime>
#include <iostream>

using namespace std;

class PID{
    private:
    void init(double P, double I, double D, double SetPoint, double current_time);
    void clear();
    double update(double feedback_value , double current_time);
    void setSampleTime(double sample_time);
    
    double Kp;
    double Ki;
    double Kd;
    double sample_time;
    double last_time;
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

void PID::init(double P, double I, double D, double SetPoint, double current_time){
    Kp = P;
    Ki = I;
    Kd = D;
    
    sample_time = 0.00;
    
    time_t ct;
    current_time = ct;
    last_time = current_time;
    
}

int main()
{
   cout << "Hello World" << endl; 
   
   return 0;
}
