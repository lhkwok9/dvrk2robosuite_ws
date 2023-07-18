#!/usr/bin/env python

import json
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import seaborn as sns

TASKNAMES = [
    'Lift',
    'PickPlace'
]

data_dir = os.path.join( os.path.dirname( os.path.dirname( os.path.abspath(__file__))), 'data_collection')

task_dirs = [os.path.join(data_dir, task_name) for task_name in TASKNAMES]

taks_files = \
{
    TASKNAMES[i]: glob.glob(task_dirs[i]) for i in range(len(TASKNAMES))
}

demonstration_files = {}

for key, val in taks_files.items():
    tmp = []
    for val_single in val:
        res = glob.glob(os.path.join(val_single, 'ep*'))
        tmp += res
    demonstration_files[key] = tmp
    
    
for key, val in demonstration_files.items():
    demonstration_files[key] = [ele + '/result.json' for ele in val]

# for key, val in demonstration_files.items():
#     for ele in val:
#         print(f'{key}: {ele} \n')
    

demon_stats = {}
data = []

for key, val in demonstration_files.items():
    time_hist = []
    for ele in val:
        with open(ele) as f:
            result = json.load(f) # "operator": str, "status": bool, "elapsed time": float, 
            if result['operator'] == 'Xiangyu Chu':
                operatorNo = 'Operator 1 '
            elif result['operator'] == 'Yunxi Tang':
                operatorNo = 'Operator 3'
            elif result['operator'] == 'Zhen Zhang':
                operatorNo = 'Operator 2'
            elif result['operator'] == 'JK':
                operatorNo = 'Operator 4'
            elif result['operator'] == 'Fiat':
                operatorNo = 'Operator 6'
            elif result['operator'] == 'Shengzhi Wang':
                operatorNo = 'Operator 5'
            else:
                level = 'Non-Expert'
            data.append(
                [key, result['elapsed time'], operatorNo, result['status']]
            )
            time_hist.append(result['elapsed time'])
    demon_stats[key] = time_hist
    

data_frame = pd.DataFrame(
    data,
    columns=['Tasks', 'Completion Time (s)', 'Operators', 'Status']
    )

sns.set()
sns.barplot(data=data_frame, x='Tasks', y='Completion Time (s)', hue='Operators') # hue='Operator'
plt.show()

# print(data_frame.iloc[0:237,1].std())
 
