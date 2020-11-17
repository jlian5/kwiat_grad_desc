import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt

#---------- Initialising Powermeter reading & USB connections -----------#
rm = visa.ResourceManager()
inst = rm.open_resource('USB0::0x1313::0x8078::P0025003::INSTR', timeout=0)
power_meter = ThorlabsPM100(inst=inst)

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
# upperTop.move_home(False)
# upperBtm.move_home(False)
# lowerTop.move_home(False)
# lowerBtm.move_home(False)
#--------------------------------------------------------------------------------------------

N: int = 10

def timeAvgRead(n : int) -> float:
    tempSum : float = 0.0
    for i in range(n):
        tempSum += power_meter.read
        time.sleep(.01)
    return tempSum / n

def moveUpper(step : float) -> bool:
    old_avg : float = timeAvgRead(N)

    upperTop.move_by(step, True)
    if(timeAvgRead(N) > old_avg):
        return True
    upperTop.move_by(-step, True)

    upperBtm.move_by(step, True)
    if(timeAvgRead(N) > old_avg):
        return True
    upperBtm.move_by(-step, True)
    return False

def moveLower(step : float) -> bool:
    old_avg : float = timeAvgRead(N)

    lowerTop.move_by(step, True)
    if(timeAvgRead(N) > old_avg):
        return True
    lowerTop.move_by(-step, True)

    lowerBtm.move_by(step, True)
    if(timeAvgRead(N) > old_avg):
        return True
    lowerBtm.move_by(-step, True)
    return False

def walkTop(step : float) ->bool:
    old_avg : float = timeAvgRead(N)

    #move up upper top, then check lower
    upperTop.move_by(step, True)
    intm_avg : float = timeAvgRead(N)
    iterations : int = 1
    lowerTop.move_by(-step/2, True)
    cur_avg : float = timeAvgRead(N)
    while(cur_avg > intm_avg):
        intm_avg = cur_avg
        lowerTop.move_by(-step/2, True)
        iterations = iterations + 1
        cur_avg = timeAvgRead(N)
    lowerTop.move_by(step/2, True)
    iterations = iterations + 1
    if(intm_avg > old_avg):
        return True
    lowerTop.move_by(step/2 * iterations, True)
    upperTop.move_by(-step, True)

    #move down upper top, then check lower
    upperTop.move_by(-step, True)
    intm_avg : float = timeAvgRead(N)
    iterations : int = 1
    lowerTop.move_by(step/2, True)
    cur_avg : float = timeAvgRead(N)
    while(cur_avg > intm_avg):
        intm_avg = cur_avg
        lowerTop.move_by(step/2, True)
        iterations = iterations + 1
        cur_avg = timeAvgRead(N)
    lowerTop.move_by(-step/2, True)
    iterations = iterations + 1
    if(intm_avg > old_avg):
        return True
    lowerTop.move_by(-step/2 * iterations, True)
    upperTop.move_by(step, True)

    return False

def walkBtm(step : float) -> bool:
    old_avg : float = timeAvgRead(N)

    #move up upper btm, then check lower
    upperBtm.move_by(step, True)
    intm_avg : float = timeAvgRead(N)
    iterations : int = 1
    lowerBtm.move_by(-step/2, True)
    cur_avg : float = timeAvgRead(N)
    while(cur_avg > intm_avg):
        intm_avg = cur_avg
        lowerBtm.move_by(-step/2, True)
        iterations = iterations + 1
        cur_avg = timeAvgRead(N)
    lowerBtm.move_by(step/2, True)
    iterations = iterations + 1
    if(intm_avg > old_avg):
        return True
    lowerBtm.move_by(step/2 * iterations, True)
    upperBtm.move_by(-step, True)

    #move down upper Btm, then check lower
    upperBtm.move_by(-step, True)
    intm_avg : float = timeAvgRead(N)
    iterations : int = 1
    lowerBtm.move_by(step/2, True)
    cur_avg : float = timeAvgRead(N)
    while(cur_avg > intm_avg):
        intm_avg = cur_avg
        lowerBtm.move_by(step/2, True)
        iterations = iterations + 1
        cur_avg = timeAvgRead(N)
    lowerBtm.move_by(-step/2, True)
    iterations = iterations + 1
    if(intm_avg > old_avg):
        return True
    lowerBtm.move_by(-step/2 * iterations, True)
    upperBtm.move_by(step, True)

    return False





# e : float = 0.001
# dif : float = 100
res : int =  .03
while(moveUpper(res) or moveLower(res)): 
    print(timeAvgRead(10))

while(walkTop(res)): 
    print(timeAvgRead(10))

while(walkBtm(res)): 
    print(timeAvgRead(10))


