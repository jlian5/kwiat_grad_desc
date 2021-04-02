#https://pypi.org/project/ThorlabsPM100/#description

import pyvisa as visa
import time
from ThorlabsPM100 import ThorlabsPM100
import signal
rm = visa.ResourceManager()
pm_rs = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
pm = ThorlabsPM100(inst=pm_rs)

import numpy as np
Ns = np.arange(10, 100, 10)
times = np.linspace(0.0, 0.05, 21)

NUM_SAMPLES = 20
sigmas = np.zeros((len(Ns), len(times)))

def getAvg(n : int, time_btn_read : float):
    total = 0.0
    for count in range(n):
        time.sleep(time_btn_read)
        total += pm.read
        
    # print(total/n)
    return total / n

# for i in range(len(Ns)):
#     print(f"i: {i} of {len(Ns)}")
#     n = Ns[i]
#     for j in range(len(times)):
#         print(f"\tj: {j} of {len(times)}")
#         t = times[j]
#         data = np.array([getAvg(n, t) for i in range(NUM_SAMPLES)])
#         sigmas[i][j] = np.std(data)
#         print(sigmas[i][j])

# import matplotlib.pyplot as plt
# import seaborn as sns
# ax = sns.heatmap(sigmas)
# ax.plot()
# plt.show()
ms = []
def sig_handle():
    global ms
    print(len(ms))

signal.signal(signal.SIGALRM, sig_handle)
signal.alarm(0.1)
while True:
    ms += pm.read



            