/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h" //絶対必要
#include "app.h"    //絶対必要
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
/*
#define kpc 1.3
#define tc 2
// method parameter configuration

#define kp 0.6 * kpc
#define ki kp/(0.5 * tc)
#define kd kp * 0.125 * tc
*/

// manual parameter configuration

#define kp 1
#define ki 0
#define kd 0

#define MAX_MIN 100
/*configure sensor and motor*/
static const sensor_port_t
    touch_sensor = EV3_PORT_1,
    color_sensor = EV3_PORT_2,
    sonar_sensor = EV3_PORT_3,
    gyro_sensor = EV3_PORT_4;

static const motor_port_t
    left_motor = EV3_PORT_C,
    right_motor = EV3_PORT_B;

#define ARR_MEMBER 4
#define KPC 0
#define TC 1
#define POWER 2
#define TIMELIMIT 3

/*
例
#define ARR_SECTION 3
double arr[ARR_SECTION][ARR_MEMBER] = {{0.00, 0.00, 100, 5},
                                        {0.00, 0.00, 100, 15},
                                         {0.00, 0.00, 100, 20}};
*/

#define ARR_SECTION 9 //区間数によってかわる
                      // KPC, TC, POWER, TIME(s)
double arr[ARR_SECTION][ARR_MEMBER] = {{1.28, 2, 100, 5.5},
                                       {1.144, 2.6, 75, 10},
                                       {1.1, 2.42, 100, 13.0},
                                       {1.144, 2.35, 70, 21},
                                       {1.385, 2.45, 80, 22},
                                       {1.142, 2.42, 100, 28.5},
                                       {1.6, 2.6, 70, 50.5},
                                       {1.142, 2.42, 100, 32},
                                       {1.142, 2.42, 25, 55}};

/*declare global variable*/
double ref;
int feed;
int timestamp;
double current_time;
double start_time;
double delta_time;
double out_time;
int ctr;
/*main task*/
void main_task(intptr_t unused)
{

    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    PID pid = PID(SP, 0.00);
    pid.setRefMinMax(2, 33);

    size_t pos = 0;

    act_tsk(SUB_TASK);
    tslp_tsk(1000);

    while (1)
    {
        // tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* タッチセンサが押された */
        }

        tslp_tsk(10 * 1000U); /* 10msecウェイト */
    }

    timestamp = getTime();
    current_time = (double)timestamp / 1000000;
    start_time = current_time;

    /*write code here*/
    while (1)
    {
        /*get cur time*/
        timestamp = getTime();
        tslp_tsk(4 * 1000U);
        current_time = (double)timestamp / 1000000;
        delta_time = current_time - start_time;
        out_time = delta_time * 100;

        if (arr[pos][TIMELIMIT] >= delta_time)
        {
            /*PID control*/
            ref = ev3_color_sensor_get_reflect(EV3_PORT_2);
            pid.parameter(arr[pos][KPC], arr[pos][TC]);
            feed = (int)pid.update(ref, current_time);

            if (feed > 100)
            {
                feed = 100;
            }
            else if (feed < -100)
            {
                feed = -100;
            }

            if (feed < 15 && feed > -15)
            {
                feed = 0;
            }
            ev3_motor_steer(left_motor, right_motor, (int)arr[pos][POWER], feed);
            /*smoothig motor movement*/
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
        else
        {
            pid.clear();
            pos = pos + 1;
            if (pos >= ARR_SECTION)
            {
                break;
            }
        }
    }
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);
    ext_tsk();
}

void sub_task(intptr_t unused)
{
    while (1)
    {
        if (current_time != 0.00)
            syslog(LOG_NOTICE, "%d %d %d ms", (int)current_time, (int)start_time, (int)out_time);
        tslp_tsk(100 * 1000);
    }
}

SYSTIM getTime()
{
    static SYSTIM start = -1;
    SYSTIM time;
    SYSTIM now;

    get_tim(&time);

    if (start < 0)
    {
        start = time;
    }

    now = time - start;
    return now;
}