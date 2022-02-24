# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 10:40:54 2022

@author: dmulr
"""

import os
import pathlib
from shutil import copyfile
import csv
import time
import numpy as np
from datetime import datetime


#### SIMULATION MODES ####
collect_data = True
timestep = .0005
tend = 5
save_rate = 100
end = int(tend/timestep) 
path = os.path.dirname(__file__)
mainDirectory = path

'''
Icosphere42.obj
162_v3.obj
642_.25r.obj
'''

#### ICOSPHERE GEOMETRY ####
shape = '162_v3.obj'
shape_path = "F:/Soro_chrono/python/PyBullet/3-D_strings/Blender/"
icosphere_radius = 0.25 # Made in Blender (c). DO NOT EDIT!!
springElasticStiffness = 100
springDampingStiffness = 1
useBendingSprings = 100
frictionCoeff = 0.2
nb=162
#data = np.load(str(nb)+'.npz')

#nb=data['bot_number']

#robot_indexes = [75, 20, 91, 108, 130, 138, 39, 83, 28, 158, 135, 118]
#### INTERIOR GEOMETRY ####
particle_radius = .01
particle_width = 2*particle_radius
particle_mass = .01
xc = 0
yc = 0
zc = -.25
r_ = icosphere_radius-particle_width
segs = 12
xsegs = np.linspace(-r_,r_,segs)
rad_seg = np.sqrt(((0.14)**2)*(1-(xsegs**2/(r_**2))))


#### CONTROL SPHERE 


control_sphere_radius = .005
control_sphere_width = 2*control_sphere_radius
control_sphere_mass = .01
    
    
    
#### CONTROL MODE ####
'''
Sphere
Cube
None
Pryamid
'''
force_mag = 30# for control balls. You CAN edit this.
beta = 1
des_rad = .25
desired_shape = 'Pryamid'
theta=np.pi/10

now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
savefile = mainDirectory +'/Experiments/'+ dt_string
os.makedirs(savefile, exist_ok=True)
txtFile = savefile+'/Parameters.csv'


##### export as npy file
envParams = {}
envParams['collect_data'] = collect_data
envParams['timestep'] = timestep
envParams['tend'] = tend
envParams['save_rate'] = save_rate
envParams['end'] = end 
envParams['path'] = path
envParams['mainDirectory'] = mainDirectory
envParams['nb'] = nb


envParams['shape'] = shape
envParams['shape_path'] = shape_path
envParams['icosphere_radius'] = icosphere_radius # Made in Blender (c). DO NOT EDIT!!
envParams['springElasticStiffness'] = springElasticStiffness
envParams['springDampingStiffness'] = springDampingStiffness
envParams['useBendingSprings'] = useBendingSprings
envParams['frictionCoeff'] = frictionCoeff

envParams['particle_radius'] = particle_radius
envParams['particle_width'] = particle_width
envParams['particle_mass'] = particle_mass

envParams['control_sphere_mass'] = control_sphere_mass
envParams['control_sphere_radius'] = control_sphere_radius
envParams['control_sphere_width'] = control_sphere_width

envParams['xc'] = xc
envParams['yc'] = yc
envParams['zc'] = zc
envParams['r_'] = r_
envParams['segs'] = segs
envParams['xsegs'] = xsegs
envParams['rad_seg']= rad_seg

envParams['force_mag'] = force_mag 
envParams['beta'] = beta
envParams['des_rad'] = des_rad
envParams['desired_shape'] = desired_shape
envParams['theta']=theta
np.save(savefile+'/Parameters.npy',envParams)


