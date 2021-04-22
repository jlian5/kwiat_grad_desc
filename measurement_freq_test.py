#https://pypi.org/project/ThorlabsPM100/#description

import pyvisa as visa
import time
from ThorlabsPM100 import ThorlabsPM100
import signal
rm = visa.ResourceManager()
pm_rs = rm.open_resource('USB::0x1313::0x8078::P0027638::INSTR', timeout=0)
pm = ThorlabsPM100(inst=pm_rs)
import numpy as np
import pandas as pd


def getAvg(n : int, gap : int, data: list):
    total = 0.0

    idx = 0
    count = 0
    while count < n:
        total += data[idx]
        idx += gap
        count += 1
    
    # print(total/n)
    return total / n

#.150-.200 mW

def main():
    Ns = np.flip(np.arange(10, 100, 10))
    
    # times = np.linspace(0.0, 0.05, 21)
    gaps = np.array([1, 2, 3, 5, 10, 20, 30, 40, 50 ,60, 70, 80, 90,100,150,200])
    # gaps = np.array([1, 2, 3, 5, 10, 20])

    NUM_SAMPLES = 50
    measurement_data = []
    for sample_i in range(NUM_SAMPLES):
        print(sample_i)
        sample = []
        for itr in range(gaps[-1] * Ns[0]):
            sample.append(pm.read)
        measurement_data.append(sample)

    percent = np.zeros((len(Ns), len(gaps)))
    sigmas = np.zeros((len(Ns), len(gaps)))
    for i in range(len(Ns)):
        print(f"i: {i} of {len(Ns)}")
        n = Ns[i]
        for j in range(len(gaps)):
            print(f"\tj: {j} of {len(gaps)}")
            gap = gaps[j]
            data_i = np.array([getAvg(n, gap, measurement_data[i]) for i in range(NUM_SAMPLES)])
            sigmas[i][j] = np.std(data_i)
            percent[i][j] = sigmas[i][j] / np.mean(data_i)
            # print(sigmas[i][j])

    import matplotlib.pyplot as plt
    import seaborn as sns
    df = pd.DataFrame(percent, columns=gaps, index=Ns)
    # df = df.pivot("gap b/n measurement", "# of measurement", "std.dev of 20 measurements")
    ax = sns.heatmap(df,annot=True, fmt=".3g")
    ax.set(xlabel="gap b/n measurement", ylabel='# of measurement')
    ax.plot()

    plt.show()

    return

            
if __name__ == "__main__":
    main()