#!/usr/bin/python

import pandas as pd 
import sys

# custom imports
from data_handling import *
from plotting import *

def main():
    cpu_file = sys.argv[1]
    print("Analyzing CPU data from :", cpu_file)
    data = read_data(cpu_file)
    plot_boxplot(data)

  
  
if __name__=="__main__":
    main()