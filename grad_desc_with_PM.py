import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt
import inspect
from typing import Callable
import signal
import sys


#TODO Record start positions of each and log to file
#TODO variable step size requires keeping track of previous steps, implement

#TODO TEST DEBUGGING STATEMENTS

#---------- Initialising Powermeter reading & USB connections -----------#
rm = visa.ResourceManager()
p2Res = rm.open_resource('USB0::0x1313::0x8078::P0025003::INSTR', timeout=0)
p3Res = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
p3 = ThorlabsPM100(inst=p3Res)
p2 = ThorlabsPM100(inst=p2Res)
ratio: float = .9815 #measured before hand p3 / p1 where p1 is the free space measurement before measurement fiber

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

upperTopStart = 5.08984
upperBtmStart = 5.55010
lowerTopStart = 6.87527
lowerBtmStart = 4.90438
# upperTop.move_to(upperTopStart,True)
# upperBtm.move_to(upperBtmStart,True)
# lowerTop.move_to(lowerTopStart,True)
# lowerBtm.move_to(lowerBtmStart,True)
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
    time.sleep(.1)
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

def my_getRes(baseRes : float = 0.005) -> float:
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
    upperTop.move_to(target["ut"], True)
    upperBtm.move_to(target["ub"], True)
    lowerTop.move_to(target["lt"], True)
    lowerBtm.move_to(target["lb"], True)

def optimize_direction(knob, getRes : Callable[[float],float], forward: bool) -> int:
    multiplier : int = 1 if forward else -1
    iterations : int = 0
    init_reading : float = timeAvgRead(N)
    print(f"\t\t initial {init_reading}")
    while True:
        intm_pos : dict = get_curr_config()
        curr_reading : float = timeAvgRead(N)
        knob.move_by( multiplier * getRes(), True)
        # print(f"timeavg { timeAvgRead(N)} initial {init_reading}")
        if timeAvgRead(N) < curr_reading:
            reset_config(intm_pos)
            break
        else:
            iterations += 1
    print(f"\t\t final {timeAvgRead(N)}")
    return iterations

def optimize_knob(knob,getRes : Callable[[float],float]) -> int:
    forward: int = optimize_direction(knob, getRes, True)
    if(forward > 0):    
        print(f"\t moved forward {forward} times")
        return forward
    backward : int = optimize_direction(knob, getRes, False)
    print(f"\t moved back {backward} times")
    return backward

def plot_exit():
    import matplotlib.pyplot as plt
    plt.plot(reading)
    plt.xlabel('# of motor increments')
    plt.ylabel('normalized coupling efficiency')
    plt.show()

def signal_handler(sig, frame):
    plot_exit()
    sys.exit(0)
#-----------
def moveUpper(getRes : Callable[[float],float]) -> bool:
    print("\nMoving upper top knob")
    top: int = optimize_knob(upperTop, getRes)
    print(f"reading is {timeAvgRead(N)}")


    print("Moving upper bottom knob")
    btm: int = optimize_knob(upperBtm,getRes)
    print(f"reading is {timeAvgRead(N)}")
    return (top > 0 or btm > 0)

def moveLower(getRes : Callable[[float],float]) -> bool:
    print("\nMoving lower top knob")
    top: int = optimize_knob(lowerTop, getRes)
    print(f"reading is {timeAvgRead(N)}")


    print("Moving lower bottom knob")
    btm: int = optimize_knob(lowerBtm,getRes)
    print(f"reading is {timeAvgRead(N)}")
    return (top > 0 or btm > 0)

walkTopMode: str = ""
def walkTop(getRes : Callable[[float],float]) ->bool:
    global walkTopMode
    print(f"\nwalking top with starting efficiency {timeAvgRead(N)}")
    top_knob_init : dict = get_curr_config()
    if(walkTopMode == "" or walkTopMode == "forward"):
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*getRes(), True) #move up lower top, 
        optimize_knob(upperTop, getRes)
        # optimize_direction(upperTop, getRes, True)

        if(timeAvgRead(N) > old_avg):
            walkTopMode = "forward"
            print(f"Finished walking top with efficiency {timeAvgRead(N)}")
            return True
        else:
            # walkTopMode = "" 
            reset_config(top_knob_init)

    if(walkTopMode == "" or walkTopMode == "backward"):
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*-getRes(), True) #move down lower top, then check upper
        optimize_knob(upperTop, getRes)
        # optimize_direction(upperTop, getRes, False)

        if(timeAvgRead(N) > old_avg):
            walkTopMode = "backward"
            print(f"Finished walking top with efficiency {timeAvgRead(N)}")
            return True
        else:
            reset_config(top_knob_init)
    
    walkTopMode = ""
    print(f"Failed walking top with efficiency {timeAvgRead(N)}")
    return False

walkBtmMode: str = ""
def walkBtm(getRes : Callable[[float],float]) -> bool:
    global walkBtmMode
    print(f"\nwalking btm with starting efficiency {timeAvgRead(N)}")
    btm_knob_int : dict = get_curr_config()
    if(walkBtmMode == "" or walkBtmMode == "forward"):
        old_avg : float = timeAvgRead(N)
        
        lowerBtm.move_by(1.5*getRes(), True) #move up lower btm, 
        optimize_knob(upperBtm, getRes)
        # optimize_direction(upperTop, getRes, True)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "forward"
            print(f"Finished walking btm with efficiency {timeAvgRead(N)}")
            return True
        else:
            # walkBtmMode = ""
            reset_config(btm_knob_int)
        
    if(walkBtmMode == "" or walkBtmMode == "backward"):
        old_avg : float = timeAvgRead(N) #move down lower Btm, then check upper

        lowerBtm.move_by(1.5*-getRes(), True)
        optimize_knob(upperBtm, getRes)
        # optimize_direction(upperTop, getRes, False)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "backward"
            print(f"Finished walking btm with efficiency {timeAvgRead(N)}")
            return True
        else:
            reset_config(btm_knob_int)

    walkBtmMode = ""
    print(f"Failed walking btm with efficiency {timeAvgRead(N)}")
    return False



res : int =  .045
iterationSingle : int = 0
iterationWalk: int =2
initial : float = timeAvgRead(N)
signal.signal(signal.SIGINT, signal_handler)
print(f"initial reading is {initial}") 

default_res = lambda : res

for i in range(iterationSingle):
    moveLower(my_getRes)
    moveUpper(my_getRes)

    

for i in range(iterationWalk):
    while walkTop(my_getRes):
        pass
    while walkBtm(my_getRes):
        pass

print(f"final reading is {timeAvgRead(N)}")



plot_exit()
