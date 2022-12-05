#!/usr/bin/python

import pandas as pd 
import sys
from ruamel.yaml import YAML
import shutil


def read_data(file: str) -> pd.DataFrame:
    '''
    read comma seperated csv file and return
    data as a pandas data frame object
    '''
    return pd.read_csv(file, sep=',')

def parse_config_file(config_fn, sort_names):
    yaml = YAML()
    with open(config_fn) as f:
        d = yaml.load(f)
    datasets = d['Datasets'].keys()
    if sort_names:
        datasets = sorted(datasets)
    datasets_labels = {}
    datasets_titles = {}
    for v in datasets:
        datasets_labels[v] = d['Datasets'][v]['label']
        if 'title' in d['Datasets'][v]:
            datasets_titles[v] = d['Datasets'][v]['title']

    algorithms = d['Algorithms'].keys()
    if sort_names:
        algorithms = sorted(algorithms)
    alg_labels = {}
    alg_fn = {}
    for v in algorithms:
        alg_labels[v] = d['Algorithms'][v]['label']
        alg_fn[v] = d['Algorithms'][v]['fn']

    boxplot_distances = []
    if 'RelDistances' in d:
        boxplot_distances = d['RelDistances']
    boxplot_percentages = []
    if 'RelDistancePercentages' in d:
        boxplot_percentages = d['RelDistancePercentages']

    if boxplot_distances and boxplot_percentages:
        print(Fore.RED + "Found both both distances and percentages for boxplot distances")
        print(Fore.RED + "Will use the distances instead of percentages.")
        boxplot_percentages = []

    return datasets, datasets_labels, datasets_titles, algorithms, alg_labels, alg_fn,\
        boxplot_distances, boxplot_percentages

def read_data_from_config(config):
    cpu_usage_per_dataset = {}
    cpu_usage_per_algo = {}

def read_data_from_file(results_dir,
                        nm_file_cpu='log_cpu.csv'):

    fn_cpu = os.path.join(results_dir, nm_file_cpu)
    data_cpu = np.loadtxt(fn, delimiter=",", dtype=float)

    print(data_cpu)

    return data_cpu
