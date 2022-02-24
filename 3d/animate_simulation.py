# -*- coding: utf-8 -*-
"""
Created on Sun Jan 16 17:33:47 2022

@author: dmulr
"""
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import os
import glob
import cv2
from tqdm import tqdm
import matplotlib.font_manager as fm
from objects import *


path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
print(name)
name='15_02_2022_11_02_34'
#name='28_01_2022_13_01_00'

d=.5
xmin=-d
xmax=d

ymin=-d
ymax=d

zmin=-d
zmax=d


obj=import_data(name,path,xmin,xmax,ymin,ymax,zmin,zmax)
#obj.field_value()
#obj.create_animation()
obj.create_snapshot()
#obj.field_value()