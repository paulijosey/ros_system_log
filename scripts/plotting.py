#!/usr/bin/python

import pandas as pd 
import sys
import matplotlib.pyplot as plt
import numpy as np
import colorsys

def plot_boxplot(data: pd.DataFrame) -> None:
    '''
    Take a pandas data frame object and create a boxplot from
    it. This function is designed for CPU utilization plotting.
    '''
    # set the lables
    plt.title('CPU Analyzation')
    plt.xlabel('System Processes')
    plt.ylabel('CPU usage [%]')
    # actually plot
    bp = plt.boxplot(data, patch_artist=True)
    N = len(bp['boxes'])
    hue = [h for h in np.linspace(0, 1, N+1)]
    i = 0
    for box in bp['boxes']:
        (r, g, b) = colorsys.hsv_to_rgb(hue[i], 1.0, 1.0)
        box.set(color=[r,g,b])
        box.set(facecolor=[r,g,b])
        i = i+1
    tmp_arr = np.arange(1, N+1)
    plt.xticks(tmp_arr,data.columns.values)
    plt.show()
