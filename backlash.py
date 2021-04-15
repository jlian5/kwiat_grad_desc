import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt

rm = visa.ResourceManager()
pm_rs = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
pm = ThorlabsPM100(inst=pm_rs)

def timeAvgRead() -> float:
    tempSum : float = 0.0
    time.sleep(.001)
    for i in range(10):
        tempSum += pm.read

    return tempSum/10

upperTopSN = 27004949
# upperBtmSN = 27004956
# lowerTopSN = 27004948
# lowerBtmSN = 27004958

upperTop = apt.Motor(upperTopSN)
# upperBtm = apt.Motor(upperBtmSN)
# lowerTop = apt.Motor(lowerTopSN)
# lowerBtm = apt.Motor(lowerBtmSN)
count = np.flip(np.arange(1, 10, 2)) #backup in how many substeps
steps = np.linspace(.01, .1, 10) #go forward how much

NUM_SAMPLES = 5

mean_diff = np.zeros((len(count), len(steps)))
std_dev  = np.zeros((len(count), len(steps)))
position = upperTop.position

for i in range(len(count)):
    print(f"i is {i} of {len(count)}")
    cnt = count[i]
    for j in range(len(steps)):
        print(f"\t\tj is {j} of {len(steps)}")
        step = steps[j]
        data = []
        for sample in range(NUM_SAMPLES):
            upperTop.move_to(position, True)
            begin_read = timeAvgRead()
            upperTop.move_by(step,True)
            for step_cnt in range(cnt):
                upperTop.move_by(-step/cnt,True)
            end_read = timeAvgRead()
            data.append(end_read - begin_read)
        mean_diff[i][j] = np.mean(np.array(data))
        std_dev[i][j] = np.std(np.array(data))

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
df_mean = pd.DataFrame(mean_diff, columns=steps, index=count)
df_std = pd.DataFrame(std_dev, columns=steps, index=count)
# df = df.pivot("gap b/n measurement", "# of measurement", "std.dev of 20 measurements")
ax_mean = sns.heatmap(df_mean,annot=True, fmt=".3g")
ax_mean.set(xlabel="backup in how many substeps", ylabel='forward increment')
ax_mean.set_title("mean diff")
ax_mean.plot()
plt.show()
plt.clf()
ax_std = sns.heatmap(df_std,annot=True, fmt=".3g")
ax_std.set(xlabel="backup in how many substeps", ylabel='forward increment')
ax_std.set_title("std.dev of diff")
ax_std.plot()

plt.show()