import numpy as np
import os, sys

data_directory = os.path.join( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ), 'data_collection')

success_info = np.load(
    os.path.join( data_directory, 'ep_1688013646_1453464/state_1688013652_8820388/successful.npy' )
)

print(success_info)