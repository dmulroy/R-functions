# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

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
dimension = '2D' #2D: 2D sim   3D: 3D sim
dt = 0.001 # time step 
time_end = 1
save_rate = 50 #save every n number of steps
visual = 'pov'

xcenter = 0
zcenter = 0

#### Control Modes ####
'''
Control modes:
shape formation: Control mode specific to shape formation
shape morphing linear: linear shape morphing 
shape_morphing: transfinte morphing 
'''
 
control_mode = "shape_formation"

#### GEOMETRIES ####
"""
circle:
square:
wrench
"""

#### CONVERSION VARIABLES ####
'''
cm: convert_dist = 100
meters: convert_dist = 1

kg: convert mass = 1
grams: convert mass = 1000

'''
convert_dist = 1 # if its meters or cm
convert_mass = 1 # if its grams or kg


#### ROBOT VARIABLES #####
nb = 30 # number of bots
bot_mass = .200  # mass of bot kg 
bot_geom = 'cylinder'
bot_width = 0.07  #  bot width  [m]
bot_height = 0.07/2  # bot height    [m]
membrane_type = 1
skin_width = 0.03 # diameter of membrane particles
ns = 7 # number of skin particles 
membrane_width = ns*skin_width # cloth width [m]
spring_stiffness = 75 # spring stiffness
spring_damping = 0 # spring damping 
theta = 2*np.pi/nb 
cord_length = membrane_width + bot_width*np.cos(theta/2) # Cord length between bot centers
R = np.sqrt((cord_length**2)/(2*(1-np.cos(theta)))) # radius [m]





#### INTERIOR PROPERTIES ####
'''

"bi_dispersion_ring"
"bidispersion"
"monodispersion"
"bi_dispersion_uniform_ring"
'''



particle_mass = 0.01 # kg 
particle_width = 0.06 # meters
particle_height = bot_height # meters
particle_geom = 'cylinder'
interior_mode = "bidispersion" # interior particle mode
scale_radius = .5
offset_radius = 0

#### FLOOR PARAMETERS ####
floor_length=100 # Length of the body floor
floor_height=particle_height     # height of the body floor


#### ENVIROMENT PARAMETERS ####
lateralFriction = 0.2
spinningFriction = 0.01
rollingFriction = 0.001
dampingterm = 0.000
Ct = 0.00000 # tangent compliane
C = 0.000000 # compliance
Cr = 0.00000 # rolling compliance
Cs = 0.00000 # sliding compliance


#### CONTROL MODE ####
if control_mode=="shape_formation":
    geometry = 'rectangle'
    if geometry == "circle":
        a = 1
        b = 1
        
    if geometry == 'pacman':
        a = R
        b = R
        
        
    else:
        a=0
        b=0        
    scale = 1
    alpha = 1
    beta = 0

#### CONTROL MODE ####
if control_mode=="shape_morphing":
    geometry1 = 'circle'
    geometry2 = 'wrench'
    if geometry1 == "circle":
        a = 0
        b = 0
    
    if geometry2 == "circle":
        a = 0
        b = 0    
    
    scale1 = R
    scale2 = 1
    p = 0.5
    alpha = 1
    beta = 0


#### SAVE SIMULATION ####
now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
mainDirectory = "F:/Soro_chrono/python/Pychrono/Strings/New_strings/"
savefile = mainDirectory +'Experiments/'+ dt_string
os.makedirs(savefile, exist_ok=True)
txtFile = savefile+'/Parameters.csv'
    




##### export as npy file
envParams = {}
envParams['dt'] = dt
envParams['time_end'] = time_end
envParams['save_rate'] = save_rate
envParams['convert_dist'] = convert_dist
envParams['convert_mass'] = convert_mass
envParams['visual'] = visual
envParams['xcenter'] = xcenter
envParams['zcenter'] = zcenter

# control mode
envParams['control_mode'] = control_mode
envParams['alpha'] = alpha
envParams['beta'] = beta

if control_mode=='shape_formation':
    envParams['geometry'] = geometry
    envParams['scale'] = scale
    envParams['a'] = a
    envParams['b'] = b
    
    
if control_mode=='shape_morphing':
    envParams['geometry1'] = geometry1
    envParams['geometry2'] = geometry2
    envParams['scale1'] = scale1   
    envParams['scale2'] = scale2
    envParams['p'] = p
    if geometry1=='circle' or geometry2=='circle':
        envParams['a'] = a
        envParams['b'] = b
        
        
        
# Robot Parameters
envParams['nb'] = nb 
envParams['bot_mass'] = bot_mass
envParams['bot_geom'] = bot_geom
envParams['bot_width'] = bot_width
envParams['bot_height'] = bot_height
envParams['membrane_width'] = membrane_width
envParams['skin_width'] = skin_width
envParams['membrane_type'] = membrane_type
envParams['cord_length'] = cord_length
envParams['spring_stiffness'] = spring_stiffness
envParams['spring_damping'] = spring_damping
envParams['ns'] = ns
envParams['R'] = R

# Particle Parameters  
envParams['interior_mode'] = interior_mode
envParams['particle_mass'] = particle_mass
envParams['particle_width'] = particle_width
envParams['particle_height'] = particle_height 
envParams['particle_geom'] = particle_geom
envParams['offset_radius'] = offset_radius
envParams['scale_radius'] = scale_radius
# floor parameters
envParams['floor_length'] = floor_length
envParams['floor_height'] = floor_height



# Physical Paramters
envParams['lateralFriction'] = lateralFriction
envParams['spinningFriction'] = spinningFriction
envParams['rollingFriction'] = rollingFriction
envParams['dampingterm'] = dampingterm
envParams['Ct'] = Ct
envParams['C'] = C
envParams['Cr'] = Cr
envParams['Cs'] = Cs
envParams['number_parameters'] = len(envParams)

np.save(savefile+'/Parameters.npy',envParams)




#data=np.load(savefile+'/Parameters.npy',allow_pickle=True)
#data=data.tolist()