#!/usr/bin/env pybricks-micropython
import PID
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Button, Color, ImageFile, SoundFile
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from time import sleep

import time

#setup
sensor = ColorSensor(Port.S2)
left_motor = Motor(Port.D)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, 100, 150)
brick.sound.beep()

#switcher
class PIDSwitcher(object):
    def indirect(self,i , counter):
        method_name='number_'+str(i)
        method=getattr(self,method_name,lambda :'Invalid')
        start = time.time()
        return method(c, counter)
    def number_0(self, counter, start):                     #turn right
        test_pid(1.5, 0.5, 0.1, 80.0, start, counter)
    def number_1(self, counter, start):                     #turn left
        test_pid(0.0, 0.0, 0.0, 100.0, start, counter)
    def number_2(self, counter, start):                     #go straight
        test_pid(0.0, 0.0, 0.0, 100.0, start, counter)

#pid
def test_pid(P = 0.0,  I = 0.0, D= 0.0, L=100, T = 0.0, counter = 0.0 ):
    pid = PID.PID(P, I, D, L)
    pid.setSampleTime(0.01)
        
    while time.time() - T < counter:
        feedback = sensor.reflection()
        print(feedback)
        pid.update(feedback)
        output = pid.output
        print(output)
        robot.drive(1000, output)
        sleep(0.01)

if __name__ == "__main__":
    s = PIDSwitcher()
    s.indirect(0, 10)
