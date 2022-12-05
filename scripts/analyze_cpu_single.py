#!/usr/bin/python

import pandas as pd 
import sys
import os

# custom imports
from data_handling import *
from plotting import *

def analyze_cpu_single():
    cpu_file = sys.argv[1]
    print("Analyzing CPU data from :", cpu_file)
    data = read_data(cpu_file)
    plt = plot_boxplot(data)
    # saving 
    os.makedirs(os.path.dirname(cpu_file) + '/saved_sys_analysis', exist_ok=True)
    plt.savefig(os.path.dirname(cpu_file) + '/saved_sys_analysis/log_cpu.pdf')
  
if __name__=="__main__":
    analyze_cpu_single()