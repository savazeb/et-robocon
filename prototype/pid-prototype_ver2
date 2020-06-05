import random

#defining varible
target = 33
error = 0
integral = 0
derivative = 0
last_error = 0
delta_t = 0.004

def Pid_cal(Kp, Ki, Kd):
    global target, error, integral, derivative, last_error, delta_t
    last_error = error
    currentVal = random.randint(0,50)
    print(currentVal)
    error = target - currentVal
    integral += (error + last_error) / 2.0 * delta_t
    derivative = error - last_error / delta_t
    pCor = Kp * error
    iCor = Ki * integral
    dCor = Kd * derivative
    correction = pCor + iCor + dCor
    
    print(pCor, iCor, dCor)
    return correction

i = 0
while i < 30:
    cor_val = Pid_cal(0.83, 0.0, 0.027 )
    i = i + 1
