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
ratio: float = .9966 #measured before hand p3 / p1 where p1 is the free space measurement before measurement fiber

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
# upperTop.move_to(upperTopStart)
# upperBtm.move_to(upperBtmStart)
# lowerBtm.move_to(lowerBtmStart)
# lowerTop.move_to(lowerTopStart)
# exit()
# print("take before calibration now")
# for i in range(25):
    # print(str(i) + "...")
    # time.sleep(1)
# ----------------------

#aux variables
N: int = 10 #number of power meter reads to average

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
        input()
        print(f"caller is {inspect.stack()[1][3]}")
        print(f"Current reading: {res} @ count {len(reading)}")
    return res

def my_getRes(baseRes : float = 0.03) -> float:
    global N
    curr : float = timeAvgRead(N)
    if ((1-curr)**2 * baseRes) < .00005:
        return .00005
    return (1-curr) * baseRes


def moveUpper(getRes : Callable[[float],float]) -> bool:
    print("Moving upper top knob")
    old_avg : float = timeAvgRead(N)
    original_avg: float = old_avg
    counter: int = 0
    while True:
        upperTop.move_by(getRes(), True)
        counter = counter + 1
        tempAvg: float = timeAvgRead(N)
        if(tempAvg < old_avg): #this will always make setup move one too many iterations
            upperTop.move_by(-getRes(), True) #move back one iteration
            break
        old_avg = tempAvg
    if counter == 1: #moving top knob forwards was not the right way 
        counter = 0
        while True:
            upperTop.move_by(-getRes(), True)
            counter = counter+1
            tempAvg = timeAvgRead(N)
            if(tempAvg < old_avg):
                upperTop.move_by(getRes(), True) #move back one iteration
                break
            old_avg = tempAvg
        print(f"Moved down {counter} times")
    else:
        print(f"Moved up {counter} times")
    print(f"reading is {timeAvgRead(N)}")
    #same logic for bottom knob
    print("Moving upper bottom knob")
    old_avg = timeAvgRead(N)
    counter = 0
    while True:
        upperBtm.move_by(getRes(), True)
        counter = counter + 1
        tempAvg = timeAvgRead(N)
        if(tempAvg < old_avg):
            upperBtm.move_by(-getRes(), True)
            break
        old_avg = tempAvg
    if counter == 1:
        counter = 0
        while True:
            upperBtm.move_by(-getRes(), True)
            counter = counter+1
            tempAvg = timeAvgRead(N)
            if(tempAvg < old_avg):
                upperBtm.move_by(getRes(), True) #move back one iteration
                break
            old_avg = tempAvg
        print(f"Moved down {counter} times")
    else:
        print(f"Moved up {counter} times")

    return True

def moveLower(getRes : Callable[[float],float]) -> bool:
    print("Moving lower top knob")
    old_avg : float = timeAvgRead(N)
    counter: int = 0
    while True:
        lowerTop.move_by(getRes(), True)
        counter = counter + 1
        tempAvg: float = timeAvgRead(N)
        if(tempAvg < old_avg): #this will always make setup move one too many iterations
            lowerTop.move_by(-getRes(), True) #move back one iteration
            break
        old_avg = tempAvg
    if counter == 1: #moving top knob forwards was not the right way 
        counter = 0
        while True:
            lowerTop.move_by(-getRes(), True)
            counter = counter + 1
            tempAvg = timeAvgRead(N)
            if(tempAvg < old_avg):
                lowerTop.move_by(getRes(), True) #move back one iteration
                break
            old_avg = tempAvg
        print(f"Moved down {counter} times")
    else:
        print(f"Moved up {counter} times")
    
    #same logic for bottom knob
    print("Moving lower bottom knob")
    old_avg = timeAvgRead(N)
    counter = 0
    while True:
        lowerBtm.move_by(getRes(), True)
        counter = counter + 1
        tempAvg = timeAvgRead(N)
        if(tempAvg < old_avg):
            lowerBtm.move_by(-getRes(), True)
            break
        old_avg = tempAvg
    if counter == 1:
        counter = 0
        while True:
            lowerBtm.move_by(-getRes(), True)
            counter = counter + 1
            tempAvg = timeAvgRead(N)
            if(tempAvg < old_avg):
                lowerBtm.move_by(getRes(), True) #move back one iteration
                break
            old_avg = tempAvg
        print(f"Moved down {counter} times")
    else:
            print(f"Moved up {counter} times")

    return True

walkTopMode: str = ""
def walkTop(getRes : Callable[[float],float]) ->bool:
    global walkTopMode
    print("walking top")
    if(walkTopMode == "" or walkTopMode == "forward"):
        old_avg : float = timeAvgRead(N)

        #move up lower top, 
        lowerTop.move_by(getRes(), True)
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperTop.move_by(getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #then repeatedly move up upper
            intm_avg = cur_avg
            upperTop.move_by(getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        #when while loop breaks we moved one iteration too far
        upperTop.move_by(-getRes()/2, True)
        iterations = iterations - 1 #move back one and take one off counter
        if(intm_avg > old_avg):
            walkTopMode = "forward"
            return True
        upperTop.move_by(-getRes()/2 * iterations, True) #else we want to move fine mirror back to original
        #---
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperTop.move_by(-getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg):  #repeatedly move DOWN upper
            intm_avg = cur_avg
            upperTop.move_by(-getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        #when while loop breaks we moved one iteration too far
        upperTop.move_by(getRes()/2, True)
        iterations = iterations - 1 #move back one and take one off counter
        if(intm_avg > old_avg):
            walkTopMode = "forward"
            return True
        upperTop.move_by(getRes()/2 * iterations, True) #else we want to move fine mirror back to original
        lowerTop.move_by(-getRes(), True) #since moving fine mirror both direction does not improve, we have gone in the wrong direction for coarse

    if(walkTopMode == "" or walkTopMode == "backward"):
        #---------------------
        #move down lower top, then check upper
        old_avg : float = timeAvgRead(N)
        lowerTop.move_by(-getRes(), True)
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperTop.move_by(getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #repeatedly move UP upper
            intm_avg = cur_avg
            upperTop.move_by(getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        upperTop.move_by(-getRes()/2, True)
        iterations = iterations - 1
        if(intm_avg > old_avg):
            walkTopMode = "backward"
            return True
        upperTop.move_by(-getRes()/2 * iterations, True) #reset back to original
        #------
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperTop.move_by(-getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #repeatedly move down upper
            intm_avg = cur_avg
            upperTop.move_by(-getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        upperTop.move_by(getRes()/2, True)
        iterations = iterations - 1
        if(intm_avg > old_avg):
            walkTopMode = "backward"
            return True
        upperTop.move_by(getRes()/2 * iterations, True) #reset back to original
        #return lower to original
        lowerTop.move_by(getRes(), True)
    
    walkTopMode = ""
    return False

walkBtmMode: str = ""
def walkBtm(getRes : Callable[[float],float]) -> bool:
    global walkBtmMode
    print("walking btm")
    if(walkBtmMode == "" or walkBtmMode == "forward"):
        old_avg : float = timeAvgRead(N)
        #move up lower btm, 
        lowerBtm.move_by(getRes(), True)
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperBtm.move_by(getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #then repeatedly move up upper
            intm_avg = cur_avg
            upperBtm.move_by(getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        #when while loop breaks we moved one iteration too far
        upperBtm.move_by(-getRes()/2, True)
        iterations = iterations - 1 #move back one and take one off counter
        if(intm_avg > old_avg):
            walkBtmMode = "forward"
            return True
        upperBtm.move_by(-getRes()/2 * iterations, True) #else we want to move fine mirror back to original
        #---
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperBtm.move_by(-getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg):  #repeatedly move DOWN upper
            intm_avg = cur_avg
            upperBtm.move_by(-getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        #when while loop breaks we moved one iteration too far
        upperBtm.move_by(getRes()/2, True)
        iterations = iterations - 1 #move back one and take one off counter
        if(intm_avg > old_avg):
            walkBtmMode = "forward"
            return True
        upperBtm.move_by(getRes()/2 * iterations, True) #else we want to move fine mirror back to original
        lowerBtm.move_by(-getRes(), True) #since moving fine mirror both direction does not improve, we have gone in the wrong direction for coarse
        
    if(walkBtmMode == "" or walkBtmMode == "backward"):
        #---------------------
        #move down lower Btm, then check upper
        old_avg : float = timeAvgRead(N)
        lowerBtm.move_by(-getRes(), True)
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperBtm.move_by(getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #repeatedly move UP upper
            intm_avg = cur_avg
            upperBtm.move_by(getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        upperBtm.move_by(-getRes()/2, True)
        iterations = iterations - 1
        if(intm_avg > old_avg):
            walkBtmMode = "backward"
            return True
        upperBtm.move_by(-getRes()/2 * iterations, True) #reset back to original
        #------
        intm_avg : float = timeAvgRead(N)
        iterations : int = 1
        upperBtm.move_by(-getRes()/2, True)
        cur_avg : float = timeAvgRead(N)
        while(cur_avg > intm_avg): #repeatedly move down upper
            intm_avg = cur_avg
            upperBtm.move_by(-getRes()/2, True)
            iterations = iterations + 1
            cur_avg = timeAvgRead(N)
        upperBtm.move_by(getRes()/2, True)
        iterations = iterations - 1
        if(intm_avg > old_avg):
            walkBtmMode = "backward"
            return True
        upperBtm.move_by(getRes()/2 * iterations, True) #reset back to original
        #return lower to original
        lowerBtm.move_by(getRes(), True)

    walkBtmMode = ""
    return False



res : int =  .00005
iterationSingle : int = 1
iterationWalk: int = 2
initial : float = timeAvgRead(N)
print(f"initial reading is {initial}")

default_res = lambda : .0055

for i in range(iterationSingle):
    moveLower(default_res)
    print(f"reading is {timeAvgRead(N)}")
    moveUpper(default_res)
    print(f"reading is {timeAvgRead(N)}")

    

for i in range(iterationWalk):
    while walkTop(default_res):
        print(f"reading is {timeAvgRead(N)}")
    while walkBtm(default_res):
        print(f"reading is {timeAvgRead(N)}")

print(f"final reading is {timeAvgRead(N)}")



import matplotlib.pyplot as plt
plt.plot(reading)
plt.xlabel('# of motor increments')
plt.ylabel('normalized coupling efficiency')
plt.show()
