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

#define kpc 1.88
#define tc 4.97

#define kp 0.6 * kpc
#define ki kp/(0.5 * tc)
#define kd kp * 0.125 * tc

#define POWER 100
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

    PID pid = PID(kp, ki, kd, SP);
    pid.setRefMinMax(2,33);

    act_tsk(SUB_TASK);
    tslp_tsk(1000);

    /*write code here*/
    while(1)
    { 
        /*PID control*/
        ref = ev3_color_sensor_get_reflect(EV3_PORT_2);
        feed = (int)pid.update(ref);
        if (feed >= MAX_MIN) feed = MAX_MIN;
        else if (feed <= -MAX_MIN) feed = -MAX_MIN;
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

}