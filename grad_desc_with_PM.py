import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt
import inspect
from typing import Callable

#TODO Record start positions of each and log to file
#TODO variable step size requires keeping track of previous steps, implement

#TODO TEST DEBUGGING STATEMENTS

#---------- Initialising Powermeter reading & USB connections -----------#
rm = visa.ResourceManager()
p2Res = rm.open_resource('USB0::0x1313::0x8078::P0025003::INSTR', timeout=0)
p3Res = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
p3 = ThorlabsPM100(inst=p3Res)
p2 = ThorlabsPM100(inst=p2Res)
ratio: float = .995 #measured before hand p3 / p1 where p1 is the free space measurement before measurement fiber

#----------- Initialising mirror motors----------------------------------#
print(apt.list_available_devices())
upperTopSN = 27004949
upperBtmSN = 27004956
lowerTopSN = 27004948
lowerBtmSN = 27004958

upperTop = apt.Motor(upperTopSN)
upperBtm = apt.Motor(upperBtmSN)
lowerTop = apt.Motor(lowerTopSN)
lowerBtm = apt.Motor(lowerBtmSN)

home_velocity = 1 #homing velocity
home_position = 3 #homing offset

# upperTop.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
# upperBtm.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
# lowerTop.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
# lowerBtm.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
# upperTop.move_home(True)
# upperBtm.move_home(True)
# lowerTop.move_home(True)
# lowerBtm.move_home(True)
#--------------------------------------------------------------------------------------------
#Test step size:
smUpTop = 5.25668
smUpBtm = 4.83608
smLowTop = 6.64995
smLowBtm = 4.34177

upperTopStart = 4.88984
upperBtmStart = 4.75020
lowerTopStart = 6.86984
lowerBtmStart = 4.17010
upperTop.move_to(upperTopStart,True)
upperBtm.move_to(upperBtmStart,True)
lowerTop.move_to(lowerTopStart,True)
lowerBtm.move_to(lowerBtmStart,True)
# exit()
# print("take before calibration now")
# for i in range(25):
    # print(str(i) + "...")
    # time.sleep(1)
# ----------------------

#aux variables
N: int = 10 #number of power meter reads to average

#aux functions
#for graphing purposes
reading: list = []#the reading at the i-th increment
def timeAvgRead(n : int) -> float:
    global reading
    tempSum : float = 0.0
    for i in range(n):
        time.sleep(.03)
        tempSum += (p2.read / p3.read)

    res: float = (tempSum / n) * ratio
    reading.append(res)
    if __debug__:
        print(f"caller is {inspect.stack()[1][3]}")
        print(f"Current reading: {res} @ count {len(reading)}")
        # input()
    return res

def my_getRes(baseRes : float = 0.035) -> float:
    global N
    curr : float = timeAvgRead(N)
    if ((1-curr)**2 * baseRes) < .00005:
        return .00005
    return (1-curr)**2 * baseRes

def get_curr_config() -> dict:
    res : dict = {}
    res["ut"] = upperTop.position
    res["ub"] = upperBtm.position
    res["lt"] = lowerTop.position
    res["lb"] = lowerBtm.position
    return res

def reset_config(target : dict):
    upperTop.move_to(target["ut"])
    upperBtm.move_to(target["ub"])
    lowerTop.move_to(target["lt"])
    lowerBtm.move_to(target["lb"])

def optimize_direction(knob, getRes : Callable[[float],float], forward: bool) -> int:
    multiplier : int = 1 if forward else -1
    iterations : int = 0
    init_reading : float = timeAvgRead(N)

    while True:
        intm_pos : dict = get_curr_config()
        curr_reading : float = timeAvgRead(N)
        knob.move_by( multiplier * getRes(), True)
        print(f"timeavg { timeAvgRead(N)} initial {init_reading}")
        if timeAvgRead(N) < curr_reading:
            reset_config(intm_pos)
            break
        else:
            iterations += 1
    
    return iterations

def optimize_knob(knob,getRes : Callable[[float],float]) -> int:
    forward: int = optimize_direction(knob, getRes, True)
    if(forward > 0):    
        print(f"moved forward {forward} times")
        return forward
    backward : int = optimize_direction(knob, getRes, False)
    print(f"moved back {backward} times")
    return backward

#-----------
def moveUpper(getRes : Callable[[float],float]) -> bool:
    print("Moving upper top knob")
    top: int = optimize_knob(upperTop, getRes)
    print(f"reading is {timeAvgRead(N)}")


    print("Moving upper bottom knob")
    btm: int = optimize_knob(upperBtm,getRes)
    print(f"reading is {timeAvgRead(N)}")
    return (top > 0 or btm > 0)

def moveLower(getRes : Callable[[float],float]) -> bool:
    print("Moving lower top knob")
    top: int = optimize_knob(lowerTop, getRes)
    print(f"reading is {timeAvgRead(N)}")


    print("Moving lower bottom knob")
    btm: int = optimize_knob(lowerBtm,getRes)
    print(f"reading is {timeAvgRead(N)}")
    return (top > 0 or btm > 0)

walkTopMode: str = ""
def walkTop(getRes : Callable[[float],float]) ->bool:
    global walkTopMode
    print("walking top")
    top_knob_init : dict = get_curr_config()
    if(walkTopMode == "" or walkTopMode == "forward"):
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*getRes(), True) #move up lower top, 
        optimize_knob(upperTop, getRes)

        if(timeAvgRead(N) > old_avg):
            walkTopMode = "forward"
            return True
        else:
            # walkTopMode = "" 
            reset_config(top_knob_init)

    if(walkTopMode == "" or walkTopMode == "backward"):
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*-getRes(), True) #move down lower top, then check upper
        optimize_knob(upperTop, getRes)
        if(timeAvgRead(N) > old_avg):
            walkTopMode = "backward"
            return True
        else:
            reset_config(top_knob_init)
    
    walkTopMode = ""
    return False

walkBtmMode: str = ""
def walkBtm(getRes : Callable[[float],float]) -> bool:
    global walkBtmMode
    print("walking btm")
    btm_knob_int : dict = get_curr_config()
    if(walkBtmMode == "" or walkBtmMode == "forward"):
        old_avg : float = timeAvgRead(N)
        
        lowerBtm.move_by(1.5*getRes(), True) #move up lower btm, 
        optimize_knob(upperBtm, getRes)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "forward"
            return True
        else:
            # walkBtmMode = ""
            reset_config(btm_knob_int)
        
    if(walkBtmMode == "" or walkBtmMode == "backward"):
        old_avg : float = timeAvgRead(N) #move down lower Btm, then check upper

        lowerBtm.move_by(1.5*-getRes(), True)
        optimize_knob(upperBtm, getRes)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "backward"
            return True
        else:
            reset_config(btm_knob_int)

    walkBtmMode = ""
    return False



res : int =  .0045
iterationSingle : int = 1
iterationWalk: int = 2
initial : float = timeAvgRead(N)
print(f"initial reading is {initial}") 

default_res = lambda : res

for i in range(iterationSingle):
    moveLower(my_getRes)
    print(f"reading is {timeAvgRead(N)}")
    moveUpper(my_getRes)
    print(f"reading is {timeAvgRead(N)}")

    

for i in range(iterationWalk):
    while walkTop(my_getRes):
        print(f"reading is {timeAvgRead(N)}")
    while walkBtm(my_getRes):
        print(f"reading is {timeAvgRead(N)}")

print(f"final reading is {timeAvgRead(N)}")



import matplotlib.pyplot as plt
plt.plot(reading)
plt.xlabel('# of motor increments')
plt.ylabel('normalized coupling efficiency')
plt.show()
