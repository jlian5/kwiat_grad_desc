import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt # https://github.com/qpit/thorlabs_apt/blob/master/thorlabs_apt/core.py
import inspect
from typing import Callable
import signal
import sys


#TODO Record start positions of each and log to file
#TODO variable step size requires keeping track of previous steps, implement

#TODO TEST DEBUGGING STATEMENTS

#---------- Initialising Powermeter reading & USB connections -----------#
rm = visa.ResourceManager()
# p2Res = rm.open_resource('USB0::0x1313::0x8078::P0027255::INSTR', timeout=0)
p2Res = rm.open_resource('USB0::0x1313::0x8078::P0025384::INSTR', timeout=0)
p3Res = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
p3 = ThorlabsPM100(inst=p3Res) #p3 is the reflected port
p2 = ThorlabsPM100(inst=p2Res)#p2 is the port we are coupling into
ratio: float = .96 #measured before hand p3 / p1 where p1 is the free space measurement before measurement fiber

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

#--------------------------------------------------------------------------------------------
#Test step size:
smUpTop = 5.2263
smUpBtm = 5.5522
smLowTop = 6.7272
smLowBtm = 4.8638

upperTopStart = 5.280958652496338
upperBtmStart = 5.671262741088867
lowerTopStart = 6.619606971740723
lowerBtmStart = 4.962890625
#'ut': 5.338590145111084, 'ub': 5.657590866088867, 'lt': 6.544309616088867, 'lb': 4.964610576629639
move : bool = True

if move:
    upperTop.move_to(upperTopStart,True)
    upperBtm.move_to(upperBtmStart,True)
    lowerTop.move_to(lowerTopStart,True)
    lowerBtm.move_to(lowerBtmStart,True)
# exit()

#aux variables
N: int = 10 #number of power meter reads to average

#aux functions
#for graphing purposes
reading: list = []#the reading at the i-th increment
sigmas: list = [] #the std.dev of the i-th increment
delta_i: list = [] #change in reading at the i-th increment
debug_cnt : int = 0
def timeAvgRead(n : int) -> float:
    global reading, sigmas, delta_i, debug_cnt
    temp_readings: list= []
    time.sleep(.05)
    for i in range(n):
        temp_readings.append((p2.read / p3.read) * ratio)

    temp_readings = np.array(temp_readings)
    mean = np.mean(temp_readings)
    reading.append(mean)
    sigmas.append(np.std(temp_readings) / mean)

    if(len(reading) > 1):
        delta_i.append( (reading[-1] - reading[-2]) / reading[-2])

    if __debug__ :
        if debug_cnt == 0:
            print("-----------------DEBUG---------------------------")
            print(f"caller is {inspect.stack()[1][3]}")
            print(f"Current reading: {res} @ count {len(reading)}")
            print("-------------------------------------------------")
            debug_cnt = int(input())
        debug_cnt -= 1

    return mean

def my_getRes(baseRes : float = 0.005) -> float:
    global N
    curr : float = timeAvgRead(N)
    res = np.exp(-1/2 * (1-curr))* baseRes
    # res = (1 - curr) ** 2 * baseRes
    if (res) < .00005:
        return .00005
    return res

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
    return -backward

def plot_exit():
    import matplotlib.pyplot as plt
    # plt.plot(reading)
    err = []
    upper = []
    lower = []
    for i in range(len(reading)):
        std_dev_i = sigmas[i] * reading[i]
        err.append(std_dev_i)
        upper.append(reading[i] + std_dev_i)
        lower.append(reading[i] - std_dev_i)

    # plt.plot(upper, '--')
    # plt.plot(lower, '--')
    plt.errorbar(range(len(reading)), reading, yerr=err)
    plt.xlabel('# of motor increments')
    plt.ylabel('normalized coupling efficiency')
    plt.show()

    plt.clf()
    plt.plot(reading)
    plt.plot(upper, '--')
    plt.plot(lower, '--')
    plt.xlabel('# of motor increments')
    plt.ylabel('normalized coupling efficiency')
    plt.show()

    plt.clf()
    plt.plot(sigmas)
    plt.xlabel('# of motor increments')
    plt.ylabel('std.dev of normalized coupling efficiency')
    plt.show()

    plt.clf()
    temp_delta_i = np.array(delta_i)
    delta_i[0] = np.mean(temp_delta_i[1:])
    plt.plot(delta_i)
    plt.xlabel('# of motor increments')
    plt.ylabel('change in normalized coupling efficiency')
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
    return (abs(top) > 0 or abs(btm) > 0)

def moveLower(getRes : Callable[[float],float]) -> bool:
    print("\nMoving lower top knob")
    top: int = optimize_knob(lowerTop, getRes)
    print(f"reading is {timeAvgRead(N)}")


    print("Moving lower bottom knob")
    btm: int = optimize_knob(lowerBtm,getRes)
    print(f"reading is {timeAvgRead(N)}")
    return (abs(top) > 0 or abs(btm) > 0)

walkTopMode: str = ""
walkTopInner: str = ""
def walkTop(getRes : Callable[[float],float]) ->bool:
    global walkTopMode, walkTopInner
    print(f"\nwalking top with starting efficiency {timeAvgRead(N)}")
    top_knob_init : dict = get_curr_config()
    if(walkTopMode == "" or walkTopMode == "forward"):
        print("going forwards")
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*getRes(), True) #move up lower top, 
        walkTopInner = walk_optimize_inner(walkTopInner, upperTop, getRes)

        if(timeAvgRead(N) > old_avg):
            walkTopMode = "forward"
            print(f"Finished walking top(forwards) with efficiency {timeAvgRead(N)}")
            return True
        else:
            walkTopInner = ""
            reset_config(top_knob_init)

    if(walkTopMode == "" or walkTopMode == "backward"):
        print("going backwards")
        old_avg : float = timeAvgRead(N)

        lowerTop.move_by(1.5*-getRes(), True) #move down lower top, then check upper
        walkTopInner = walk_optimize_inner(walkTopInner, upperTop, getRes)

        if(timeAvgRead(N) > old_avg):
            walkTopMode = "backward"
            print(f"Finished walking top(backwards) with efficiency {timeAvgRead(N)}")
            return True
        else:
            walkTopInner = ""
            reset_config(top_knob_init)
    
    walkTopMode = ""
    print(f"Failed walking top with efficiency {timeAvgRead(N)}")
    return False

walkBtmMode: str = ""
walkBtmInner: str = ""
def walkBtm(getRes : Callable[[float],float]) -> bool:
    global walkBtmMode, walkBtmInner
    print(f"\nwalking btm with starting efficiency {timeAvgRead(N)}")
    btm_knob_int : dict = get_curr_config()
    if(walkBtmMode == "" or walkBtmMode == "forward"):
        print("going forwards")
        old_avg : float = timeAvgRead(N)
        
        lowerBtm.move_by(1.5*getRes(), True) #move up lower btm, 
        walkBtmInner = walk_optimize_inner(walkBtmInner, upperBtm, getRes)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "forward"
            print(f"Finished walking btm(forwards) with efficiency {timeAvgRead(N)}")
            return True
        else:
            walkBtmInner = ""
            reset_config(btm_knob_int)
        
    if(walkBtmMode == "" or walkBtmMode == "backward"):
        print("going backwards")
        old_avg : float = timeAvgRead(N) #move down lower Btm, then check upper

        lowerBtm.move_by(1.5*-getRes(), True)
        walkBtmInner = walk_optimize_inner(walkBtmInner, upperBtm, getRes)

        if(timeAvgRead(N) > old_avg):
            walkBtmMode = "backward"
            print(f"Finished walking btm(backwards) with efficiency {timeAvgRead(N)}")
            return True
        else:
            walkBtmInner = ""
            reset_config(btm_knob_int)

    walkBtmMode = ""
    print(f"Failed walking btm with efficiency {timeAvgRead(N)}")
    return False

def walk_optimize_inner(inner_record: str, knob, getRes) -> str:
    print(f"\tinner string is {inner_record}")
    if inner_record == "":
            print("\tinner string unset, finding direction now...")
            res: int = optimize_knob(knob, getRes)
            if(res > 0):
                inner_record = "forwards"
            elif(res < 0):
                inner_record = "backwards"
            else:
                inner_record = ""
            print(f"\tinner string is {inner_record}")
    else:
        if inner_record == "forwards":
            optimize_direction(knob, getRes, True)
        else:
            optimize_direction(knob, getRes, False)
    return inner_record

res : int =  .045
iterationSingle : int = 1
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
print(f"final position is {get_curr_config()}")



plot_exit()
