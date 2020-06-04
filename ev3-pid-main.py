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
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, 100, 147)
brick.sound.beep()

class K_val:
    def __init__(self):  
        self.Kp = 4
        self.Ki = 0
        self.Kd = 0

def Pid_cal():
    K_data = K_val()
    Kp_val = K_data.Kp
    Ki_val = K_data.Ki
    Kd_val = K_data.Kd

    pCor_val = pCor_cal(Kp_val)
    iCor_val = iCor_cal(Ki_val)
    dCor_val = dCor_cal(Kd_val)

    correction = pCor_val + iCor_val + dCor_val
    return correction

#PID CALCULATION
def pCor_cal(Kp):
    global target, error
    currentVal = sensor.reflection()
    error = target - currentVal
    pCor = Kp * error
    return pCor
def iCor_cal(Ki):
    global integral, error
    integral = integral + error
    iCor = Ki * integral
def dCor_cal(Kd):
    global last_error, derivative
    derivative = error - last_error
    dCor = Kd * derivative
    last_error = error
    
#loop start
while True: 
#break the program using touch sensor
#program begin here

#program end here

    cor_val = Pid_cal()
    print(cor_val)
    robot.drive(max_speed/4, cor_val)
#end loop  
