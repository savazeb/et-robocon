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
robot = DriveBase(left_motor, right_motor, 54, 100)
brick.sound.beep()

def Pid_cal(Ki, Kp, Kd):
    global target, error, integral, derivative, last_error
    currentVal = sensor.reflection()
    error = target - currentVal
    pCor = Kp * error
    integral = integral + error
    iCor = Ki * integral
    derivative = error - last_error
    dCor = Kd * derivative
    last_error = error
    correction = pCor + iCor + dCor
    return correction

#loop start
while True: 
    cor_val = Pid_cal(1.2, 0, 0)
    robot.drive(max_speed, cor_val)
#end loop
