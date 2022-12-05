#!/usr/bin/python

import pandas as pd 
import sys
import matplotlib.pyplot as plt
import numpy as np
import colorsys


def plot_boxplot(data: pd.DataFrame) -> plt:
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
    return plt

def boxplot_compare(ax, xlabels,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    n_xlabel = len(xlabels)
    leg_handles = []
    leg_labels = []
    idx = 0
    for idx, d in enumerate(data):
        w = 1 / (1.5 * n_data + 1.5)
        widths = [w for pos in np.arange(n_xlabel)]
        positions = [pos - 0.5 + 1.5 * w + idx * w
                     for pos in np.arange(n_xlabel)]
        bp = ax.boxplot(d, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], c=data_colors[idx], alpha=0)
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1

    ax.set_xticks(np.arange(n_xlabel))
    ax.set_xticklabels(xlabels)
    xlims = ax.get_xlim()
    ax.set_xlim([xlims[0]-0.1, xlims[1]-0.1])
    if legend:
        # ax.legend(leg_handles, leg_labels, bbox_to_anchor=(
            # 1.05, 1), loc=2, borderaxespad=0.)
        ax.legend(leg_handles, leg_labels)
    map(lambda x: x.set_visible(False), leg_handles)