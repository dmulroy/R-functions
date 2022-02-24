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
#name = files[-1]
#name='05_01_2022_16_58_58'
name='10_01_2022_12_08_22'
dxmin=-1.25
dxmax=1.25
dymin=-.75
dymax=.75
d=1.0
snap_shot=True
membrane=False
#dxmin,dxmax,dymin,dymax
#sim_data=sim_obj.import_data(name,path,-d,d,-d,d)
sim_data=sim_obj.import_data(name,path,-d,d,-d,d)
if snap_shot==False:
    #sim_data.create_frames(membrane)
    #sim_data.create_video()
    sim_data.plot_field_values()
    #sim_data.create_snap_shot(-1,membrane)    

else:
    # 0
    # 50
    #100
    #200
    sim_data.create_snap_shot(-1,False) 
    ##sim_data.create_snap_shot2(150)
    
#sim_data.create_snap_shot2(None)


