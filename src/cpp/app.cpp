/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h" //絶対必要
#include "app.h"//絶対必要
#include <stdio.h>
#include <stdbool.h>
#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif
#include "libcpp-test.h"
#define DEBUG
#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#include "PID.h"

#define SP 55

#define kpc 1.275
#define tc 2.49

// method parameter configuration

#define kp 0.6 * kpc
#define ki kp/(0.5 * tc)
#define kd kp * 0.125 * tc


//manual parameter configuration
/*
#define kp 1
#define ki 0
#define kd 0
*/

#define POWER 75
#define MAX_MIN 100
/*configure sensor and motor*/
static const sensor_port_t
    color_sensor    = EV3_PORT_2;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B;

/*declare global variable*/
double ref;
int feed;
double timestamp;
int ctr;
/*main task*/
void main_task(intptr_t unused) {

    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */

    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    PID pid = PID(kp, ki, kd, SP, 0.00);
    pid.setRefMinMax(2, 33);

    act_tsk(SUB_TASK);
    tslp_tsk(1000);

    /*write code here*/
    while(1)
    { 
        /*PID control*/
        timestamp = getTime() / 100000;
        ref = ev3_color_sensor_get_reflect(EV3_PORT_2);
        feed = (int)pid.update(ref, timestamp);
        ev3_motor_steer(
            left_motor,
            right_motor,
            POWER,
            feed
        );
        /*smoothig motor movement*/
        tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        
        
    }
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);
    ext_tsk();
}

void sub_task(intptr_t unused)
{
    while(1){
    syslog(LOG_NOTICE, "%f", timestamp);
    tslp_tsk(100*1000);
    }
}

SYSTIM getTime()
{
    static SYSTIM start = -1;
    SYSTIM time;
    SYSTIM now;
    
    get_tim(&time);
    
    if(start < 0){
        start = time;
    }
    now = time - start - 5000000;
    if ((int)now <= 0)
        return 0;
    return now;
}