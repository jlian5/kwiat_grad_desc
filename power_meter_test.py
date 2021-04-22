#https://pypi.org/project/ThorlabsPM100/#description

import pyvisa as visa
import time
from ThorlabsPM100 import ThorlabsPM100
rm = visa.ResourceManager()
freespaceRes = rm.open_resource('USB::0x1313::0x8078::P0025003::INSTR', timeout=0)
# freespaceRes = rm.open_resource('USB::0x1313::0x8078::P0027255::INSTR', timeout=0)
reflectedRes = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
# print(rm.list_resources('?*'))
# inst =rm.open_resource('USB0::0x0000::0x0000::DG5Axxxxxxxxx::INSTR', timeout=1)
p3 = ThorlabsPM100(inst=reflectedRes)
p1 = ThorlabsPM100(inst=freespaceRes)

sum: float = 0.0
count: int = 0
while 1:
    count = count + 1
    sum = sum + (p3.read / p1.read)
    time.sleep(.3)
    # print((pm_r.read) / pm_t.read)
    print(sum / count)