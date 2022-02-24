# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 14:18:08 2021

@author: dmulr
"""

import pychrono.core as chrono
import timeit
start=timeit.default_timer()
import objects as sim_obj
import random
import os
import csv
import glob
from IPython.display import HTML
import matplotlib.pyplot as plt
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
d=1.0
dxmin=-d
dxmax=d
dymin=-d
dymax=d

snap_shot=True
membrane=False

sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)
if snap_shot==False:
    sim_data.create_frames(membrane)
    sim_data.create_video()
    sim_data.plot_field_values()
    sim_data.create_snap_shot(-1,membrane)    

else:
    sim_data.create_snap_shot(-1,False) 
    



