#!/usr/bin/env pybricks-micropython
import PID
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Button, Color, ImageFile, SoundFile
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from time import sleep

import time

# setup
sensor = ColorSensor(Port.S2)
left_motor = Motor(Port.D)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, 100, 150)
brick.sound.beep()

# pid
def test_pid(P=0.0, I=0.0, D=0.0, L=100):
    pid = PID.PID(P, I, D, L)
    pid.setSampleTime(0.01)

    while True:
        feedback = sensor.reflection()
        print(feedback)
        pid.update(feedback)
        output = pid.output
        print(output)
        robot.drive(1000 / 2, output)
        sleep(0.01)


if __name__ == "__main__":
    test_pid(1.2, 6.0, 0.0, 50)
