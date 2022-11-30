#!/usr/bin/python

import pandas as pd 
import sys


def read_data(file: str) -> pd.DataFrame:
    '''
    read comma seperated csv file and return
    data as a pandas data frame object
    '''
    return pd.read_csv(file, sep=',')
