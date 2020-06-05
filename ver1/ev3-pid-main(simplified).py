#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Button, Color, ImageFile, SoundFile
from pybricks.tools import wait
from pybricks.robotics import DriveBase

#global variable
target = 33
error = 0
integral = 0
derivative = 0
last_error = 0
max_speed = 1000

#setup
sensor = ColorSensor(Port.S2)
left_motor = Motor(Port.D)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, 57, 115)
brick.sound.beep()

def Pid_cal(Kp, Ki, Kd):
    global target, error, integral, derivative, last_error
    currentVal = sensor.reflection()
    print(currentVal)
    error = target - currentVal
    pCor = Kp * error
    integral = integral + error
    iCor = Ki * integral
    derivative = error - last_error
    dCor = Kd * derivative
    correction = pCor + iCor + dCor
    print(pCor, iCor, dCor)
    last_error = error
    return correction

#loop start
while True: 
    cor_val = Pid_cal(0.98, 0.00001, 3)
    if cor_val > 120:
        cor_val = 120
    elif cor_val < -120:
        cor_val = -120

    robot.drive(max_speed, cor_val)
#end loop
