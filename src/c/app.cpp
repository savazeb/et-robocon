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


/*configure sensor and motor*/
static const sensor_port_t
    color_sensor    = EV3_PORT_2;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B;


/*main task*/
void main_task(intptr_t unused) {
    act_tsk(SUB_TASK);
    tslp_tsk(1000);

    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */

    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /*write code here*/
    while(1)
    { 
        
    }
    
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);
    ext_tsk();
}

void sub_task(intptr_t unused)
{
    
}
