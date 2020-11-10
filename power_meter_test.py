#https://pypi.org/project/ThorlabsPM100/#description

import pyvisa as visa
import time
from ThorlabsPM100 import ThorlabsPM100
rm = visa.ResourceManager()
inst = rm.open_resource('USB::0x1313::0x8078::P0025003::INSTR', timeout=0)
# print(rm.list_resources('?*'))
# inst =rm.open_resource('USB0::0x0000::0x0000::DG5Axxxxxxxxx::INSTR', timeout=1)
power_meter = ThorlabsPM100(inst=inst)

while 1:
    # time.sleep(1)
    print(power_meter.read)