import random

#defining varible
target = 33
error = 0
integral = 0
derivative = 0
last_error = 0

class K_val:
    def __init__(self):  
        self.Kp = 1
        self.Ki = 0.000032
        self.Kd = 0.0732

def Pid_cal():
    target = 28

    K_data = K_val()
    Kp_val = K_data.Kp
    Ki_val = K_data.Ki
    Kd_val = K_data.Kd

    pCor_val = pCor_cal(Kp_val)
    iCor_val = iCor_cal(Ki_val)
    dCor_val = dCor_cal(Kd_val)

    print(pCor_val, iCor_val, dCor_val)
    print(error, last_error)
    correction = pCor_val + iCor_val + dCor_val
    print(correction)
    return correction

def pCor_cal(Kp):
    global target, error
    currentVal = random.randint(0,50)
    error = target - currentVal
    pCor = Kp * error
    return pCor
def iCor_cal(Ki):
    global integral, error
    integral = integral + error
    iCor = Ki * integral
    return iCor
def dCor_cal(Kd):
    global last_error, derivative
    derivative = error - last_error
    dCor = Kd * derivative
    last_error = error
    return dCor
i = 0
while i < 30:
    cor_val = Pid_cal()
    i = i + 1
