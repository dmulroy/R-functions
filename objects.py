# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:29:35 2021

@author: dmulr
"""


import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import random
import os
import csv
import glob
from csv import writer
import timeit
import matplotlib.pyplot as plt
from matplotlib import animation
from IPython.display import HTML
import cv2
import matplotlib.font_manager as fm
from shutil import copyfile
 
# In[]
class robots:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor
        ###### Imported Variables #########
        self.path=path
        self.mainDirectory = self.path
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb'] # number of bots
        self.bot_mass = self.convert_mass*self.parameters['bot_mass'] 
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.bot_geom = self.parameters['bot_geom']
        self.R = self.convert_dist*self.parameters['R']
        self.bot_volume=np.pi*self.bot_height*(self.bot_width/2)**2   # calculate volume
        self.membrane_width = self.convert_dist*self.parameters['membrane_width']
        self.membrane_type = self.parameters['membrane_type']
        self.skin_width = self.parameters['skin_width']
        self.ns = self.parameters['ns']
        self.spring_stiffness = self.parameters['spring_stiffness'] 
        self.spring_damping = self.parameters['spring_damping'] 
        
        
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        
        ####### Calculated variables ######
        self.bot_density=self.bot_mass/self.bot_volume # calculate density of robot 
        self.bot_material = self.Material()
        self.membrane_density = 4000
        self.countm = 0 
        self.rl = 0

        self.fixed = False
        self.membrane_material = self.Material()
        # Colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        self.col_w = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        
        
        self.skinM = []
        self.bots = []
        self.force = []
        self.Springs = []
        
        self.bot_xposition = {}
        self.bot_yposition = {}
        self.bot_zposition = {}
        
        self.bot_xvelocity = {}
        self.bot_yvelocity = {}
        self.bot_zvelocity = {}
        
        self.bot_xForcetotal = {}
        self.bot_yForcetotal = {}
        self.bot_zForcetotal = {}
        
        
        self.bot_xForcecontact = {}
        self.bot_yForcecontact = {}
        self.bot_zForcecontact = {}
        
        
        self.skin_xposition = {}
        self.skin_yposition = {}
        self.skin_zposition = {}
        
        for i in range(self.nb):
            
            # positions
            self.bot_xposition["bot_xposition{0}".format(i)]=[]  #x position
            self.bot_yposition["bot_yposition{0}".format(i)]=[]  # y position
            self.bot_zposition["bot_zposition{0}".format(i)]=[]  # z position 
            
            
            # velocities
            self.bot_xvelocity["bot_xvelocity{0}".format(i)]=[]  # x velocity
            self.bot_yvelocity["bot_yvelocity{0}".format(i)]=[]  # y velocity
            self.bot_zvelocity["bot_zvelocity{0}".format(i)]=[]  # z velocity
            
            
            # total forces
            self.bot_xForcetotal["bot_xForcetotal{0}".format(i)]=[]  # Force x
            self.bot_yForcetotal["bot_yForcetotal{0}".format(i)]=[]  # Force y
            self.bot_zForcetotal["bot_zForcetotal{0}".format(i)]=[]  # force z
            
            
            # contact forces
            self.bot_xForcecontact["bot_xForcecontact{0}".format(i)]=[]  # Force x
            self.bot_yForcecontact["bot_yForcecontact{0}".format(i)]=[]  # Force y
            self.bot_zForcecontact["bot_zForcecontact{0}".format(i)]=[]  # force z            
        
        
        
        
            # postion 
            theta=i*2*np.pi/self.nb # set angle
            x = self.R*np.cos(theta)+self.xcenter  # set x positon 
            y = self.bot_height/2                     # set y position 
            z = self.R*np.sin(theta)+self.zcenter  # set z position 
            # create body
            #### geometry of the robots to be cylinders
            if self.bot_geom=="cylinder":
                
                bot = chrono.ChBodyEasyCylinder(self.bot_width/2, self.bot_height,self.bot_density,True,True)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z)) 
                bot.SetName("bot"+str(i)) # set name (important for contact tracking)
                bot.SetId(i)              # set id   (impoortant for contact tracking )  
                # material 
                bot.SetMaterialSurface(self.bot_material)  # set material 
                # rotate them so we can form the membrane
                rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                
                
                ##### Add force components to each bot 
                # x forces 
                botforcex = chrono.ChForce()  # create it 
                bot.AddForce(botforcex) # apply it to bot object
                botforcex.SetMode(chrono.ChForce.FORCE) # set the mode 
                botforcex.SetDir(chrono.VECT_X) # set direction 
                self.force.append(botforcex) # add to force list
                
                # y forces    
                botforcey = chrono.ChForce() # create it 
                bot.AddForce(botforcey) # apply it to bot object
                botforcey.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcey.SetDir(chrono.VECT_Y) # set direction 
                self.force.append(botforcey) # add to force list
                
                # z forces            
                botforcez = chrono.ChForce() # create it 
                bot.AddForce(botforcez) # apply it to bot object
                botforcez.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcez.SetDir(chrono.VECT_Z) # set direction 
                self.force.append(botforcez) # add to force list
                
                
                
                # set max speed (Not always needed but it helps)
                bot.SetMaxSpeed(2)
                bot.SetLimitSpeed(False)
                
            
                # make the color blue 
                bot.AddAsset(self.col_b)
                
                # zeroth robot so we know which is which in the videos
                if i==0:   
                    bot.AddAsset(self.col_p)   
                if i>=20 and i<=25:
                    bot.AddAsset(self.col_w)
                if i==22:   
                    bot.AddAsset(self.col_y)  
                
                # set collision
                bot.SetCollide(True)
                
                
                # set fixed
                bot.SetBodyFixed(self.fixed)
                
                # link to floor
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                
                # add link to the system 
                #self.my_system.AddLink(pt)
                
                # add bot to series of array 
                self.my_system.Add(bot) # add bot to system 
                self.bots.append(bot) # add bot to bot array 
                
                              
            if self.membrane_type==1:
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan((self.bot_width/2)/self.R)   # angle offset for radius of bot
                p_ang=np.arctan((self.skin_width/2)/self.R)           # angle offset for radius of skin particle
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ns+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang 
                        x=self.R*np.cos(theta)+self.xcenter # x position 
                        y=self.bot_height/2              # y position 
                        z=self.R*np.sin(theta)+self.zcenter # z position  
                        
                        # create them and set position
                        #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True) # create particle
                        skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                        skinm.SetMaterialSurface(self.membrane_material) # add material 
                        skinm.SetNoGyroTorque(True) # no gyro toruqe 
                        skinm.SetName('skin'+str(i)) # create name 
                        skinm.SetId(i) # set id 
                                    # link to floor
                        pt=chrono.ChLinkMatePlane()
                        pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        # add link to the system 
                        self.my_system.AddLink(pt)
                        # rotate them bout y axis
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        
                        
                        
                        self.countm=self.countm+1
                        # Attach springs if more then one was created    
                        if j>1:
                            ground=chrono.ChLinkSpring() # create spring 1
                            p1=0; p2=self.skin_width/2 # points where each spring is attatched 
                            p3=0; p4=-self.skin_width/2
                            h=self.bot_height/5
                            
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                            ground.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground.Set_SpringR(self.spring_damping) # set damping constant
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground.AddAsset(self.col_p) # add color 
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground) # add spring to system 
                            self.Springs.append(ground)
                            ground1=chrono.ChLinkSpring() # create spring 2
                            ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True) # link spring to particles 
                            ground1.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground1.Set_SpringR(self.spring_damping) # set damping 
                            ground1.AddAsset(self.col_p) # create color 
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground1) # add to the system                                 
                            self.Springs.append(ground1)
                        #Link to cylinder 
                        if j==1:
                            skinm.AddAsset(self.col_p) # add color
                            glue=chrono.ChLinkMateFix() # cretae fix constraint
                            glue.Initialize(skinm,self.bots[i]) # fix object to bot
                            self.my_system.AddLink(glue) # add to system 
                            # Link last particle with this bot
                            if i>=2:
                                glue=chrono.ChLinkMateFix() # create the constraint 
                                glue.Initialize(self.skinM[-1],self.bots[-1]) # add constraint 
                                self.my_system.AddLink(glue) # add to the system 
                        if j==self.ns:
                            skinm.AddAsset(self.col_p)
                        self.my_system.Add(skinm)
                        self.skinM.append(skinm)
                
            # Between this bot and first bot
            if i==self.nb-1:
                for j in range(1,self.ns+1,1):
                    # Initial postion of each particle
                    theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang
                    x=self.R*np.cos(theta)+self.xcenter
                    y=self.bot_height/2
                    z=self.R*np.sin(theta)+self.zcenter
                    self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                    self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                    self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                    self.countm=self.countm+1
                    # Create particles
                    #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                    skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True)
                    skinm.SetPos(chrono.ChVectorD(x,y,z))
                    skinm.SetMaterialSurface(self.membrane_material)
                    skinm.SetNoGyroTorque(True)
                    skinm.SetName("skin"+str(i))
                    skinm.SetId(i)
                    # rotate them
                    rotation1 = chrono.ChQuaternionD()
                    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                    skinm.SetRot(rotation1)
                    pt=chrono.ChLinkMatePlane()
                    pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    # Attach springs    
                    if j>1:
                        ground=chrono.ChLinkSpring()
                        p1=0; p2=self.skin_width/2
                        p3=0; p4=-self.skin_width/2
                        h=self.bot_height/5
                
                        ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                        ground.Set_SpringK(self.spring_stiffness)
                        ground.Set_SpringR(self.spring_damping)
                        ground.Set_SpringRestLength(self.rl)
                        ground.AddAsset(self.col_y)
                        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                        self.my_system.AddLink(ground)
                        self.Springs.append(ground)
                        ground1=chrono.ChLinkSpring()
                        ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                        ground1.Set_SpringK(self.spring_stiffness)
                        ground1.Set_SpringR(self.spring_damping)
                        ground.Set_SpringRestLength(self.rl) # set resting length 
                        ground1.AddAsset(self.col_y)
                        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                        self.my_system.AddLink(ground1)  
                        self.Springs.append(ground1)
                    #Link to cylinder
                    if j==1:
                        skinm.AddAsset(self.col_p)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(skinm,self.bots[0])
                        self.my_system.AddLink(glue)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(self.skinM[-1],self.bots[0])
                        self.my_system.AddLink(glue)
                     
                    if j==self.ns:
                        skinm.AddAsset(self.col_p)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(skinm,self.bots[1])
                        self.my_system.AddLink(glue)
                        
                    self.my_system.Add(skinm)
                    self.skinM.append(skinm)               
                
               
                
               
                
    def Material(self):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(self.lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
    
 
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.Springs,self.bots,self.force)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.nb):
            self.bot_xposition["bot_xposition"+str(i)].append(self.bots[i].GetPos().x)
            self.bot_yposition["bot_yposition"+str(i)].append(self.bots[i].GetPos().y)
            self.bot_zposition["bot_zposition"+str(i)].append(self.bots[i].GetPos().z)
            
        for i in range(self.countm):
            self.skin_xposition['skin_xposition'+str(i)].append(self.skinM[i].GetPos().x)
            self.skin_yposition['skin_yposition'+str(i)].append(self.skinM[i].GetPos().y)
            self.skin_zposition['skin_zposition'+str(i)].append(self.skinM[i].GetPos().z)
            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.nb):
            self.bot_xvelocity["bot_xvelocity"+str(i)].append(self.bots[i].GetPos_dt().x)
            self.bot_yvelocity["bot_yvelocity"+str(i)].append(self.bots[i].GetPos_dt().y)
            self.bot_zvelocity["bot_zvelocity"+str(i)].append(self.bots[i].GetPos_dt().z)
            
                 # total forces
            self.bot_xForcetotal["bot_xForcetotal".format(i)]=[]  # Force x
            self.bot_yForcetotal["bot_yForcetotal".format(i)]=[]  # Force y
            self.bot_zForcetotal["bot_zForcetotal".format(i)]=[]  # force z
            
            
            # contact forces
            self.bot_xForcecontact["bot_xForcecontact".format(i)]=[]  # Force x
            self.bot_yForcecontact["bot_yForcecontact".format(i)]=[]  # Force y
            self.bot_zForcecontact["bot_zForcecontact".format(i)]=[]  # force z               
            
    # save force data            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcetotal["bot_xForcetotal"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.bot_yForcetotal["bot_yForcetotal"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.bot_zForcetotal["bot_zForcetotal"+str(i)].append(self.bots[i].Get_Xforce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcecontact["bot_xForcecontact"+str(i)].append(self.bots[i].GetContactForce().x)
            self.bot_yForcecontact["bot_yForcecontact"+str(i)].append(self.bots[i].GetContactForce().y)
            self.bot_zForcecontact["bot_zForcecontact"+str(i)].append(self.bots[i].GetContactForce().z)
            
            
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.bot_xposition,self.bot_yposition,self.bot_zposition)
    
    # return position membrane data
    def return_position_membrane_data(self):
        ''' return the dictionary of each pmembrane particle '''
        return(self.skin_xposition,self.skin_yposition,self.skin_zposition)  
      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity)
        
    # return force data    
    def return_force_data(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal)
  
    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact)
    
    
    
class Interiors:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor    
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']

        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb']
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.particle_mass = self.convert_mass*self.parameters['particle_mass']
        self.particle_width = self.convert_dist*self.parameters['particle_width']
        self.particle_height = self.convert_dist*self.parameters['particle_height']
        self.particle_geom = self.parameters['particle_geom']
        #self.R = self.convert_dist*self.parameters['R']
        self.scale_radius = self.parameters['scale_radius']
        self.R =  self.scale_radius*self.convert_dist*self.parameters['R']
        
        self.interior_mode = self.parameters['interior_mode']
        self.offset_radius=self.parameters['offset_radius']
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        
        self.particle_volume=np.pi*self.particle_height*(self.particle_width/2)**2   # calculate volume
        self.particle_density=self.particle_mass/self.particle_volume # calculate density of robot 
        self.particle_material = self.Material()
        
        self.fixed = False
        (self.n,self.Ri) = self.MaxValues()
        self.total_particles = np.sum(self.n)
        self.particle_material = self.Material()
        
        
        self.parameters['n'] = self.n
        self.parameters['total_particles'] = self.total_particles
        self.parameters['Ri'] = self.Ri
        
        
        self.number_parameters = self.parameters['number_parameters']
        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+4:
            #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
            
        else:
             print('not save')
        
        
        

        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))
        
        
        
        
        # with open(self.mainDirectory+name+"/Parameters.csv", 'a') as f_object:
        #     # Pass this file object to csv.writer()
        #     # and get a writer object
        #     writer_object = writer(f_object)
        #     #Pass the list as an argument into
        #     #the writerow()
        #     writer_object.writerow(['ring configuration',self.n])
        #     writer_object.writerow(['total number of particles',self.total_particles])
        #     #Close the file object
        #     f_object.close()        
        
        
        
        self.particle_xposition = {}
        self.particle_yposition = {}
        self.particle_zposition = {}
        
        
        self.particle_xvelocity = {}
        self.particle_yvelocity = {}
        self.particle_zvelocity = {}
        
        
        self.particle_xForcetotal = {}
        self.particle_yForcetotal = {}
        self.particle_zForcetotal = {}
        
        
        self.particle_xForcecontact = {}
        self.particle_yForcecontact = {}
        self.particle_zForcecontact = {}        
        
        
        self.particles = []
        self.Rm = []
        self.Area = 0
        self.N1=0
        self.N2=0
        
        # colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_r = chrono.ChColorAsset(); self.col_r.SetColor(chrono.ChColor(1, 0, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple        
        
        # empty array
        self.particle_xposition = {}
        self.particle_yposition  = {}
        self.particle_zposition  = {}
        
        self.particle_xvelocity = {}
        self.particle_yvelocity  = {}
        self.particle_zvelocity  = {}
        


        #### mono-dispersion #####
        if self.interior_mode=="monodispersion":  
            count=0
            for i in range(self.n.size):
                
                if i%2==0:
                    con='b'
                    const=0
                else:

                    con='a'
                    const=0
                    
                    
                self.radius2=self.particle_width/2 - self.offset_radius
                
                R2=self.radius2*self.n[i]/(np.pi) + const   
                
                for j in range(self.n[i]):
                     
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    

                    count=count+1
                    
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("grana"+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(self.col_r)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)   


        #### bidispersion 
        if self.interior_mode=="bidispersion": 
            count=0
            for i in range(len(self.n)):
                #print("i=",str(i))
                
                if i%2==0:
                    self.radius2 = (self.particle_width/2)*(2**.5)
                    con = 'b'
                    const = 0
                    self.N1 = 1+self.N1
                else:
                    self.radius2 = self.particle_width/2 
                    con = 'a'
                    const = 0
                    self.N2 = 1+self.N2
                    
                    
                R2=self.radius2*self.n[i]/(np.pi) + const
                # empty arrays of variables
                for j in range(self.n[i]):
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 
                    
                    #R2=self.radius2*self.n[i]/(np.pi) + const# raidus of ring 
                    # x,y,z positions
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter
                    y=.5*self.particle_height 
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter
                    #print("j=",str(j),str(np.round(self.radius2,3)),"x,y",str(np.round(x,2)),str(np.round(z,2)))
                    self.Rm.append(self.radius2)
                    self.Area = self.Area + (np.pi)*(self.radius2)**2
                    if i%2==0:
                        con = 'b'
                        const = 0
                        self.N1 = 1+self.N1
                    else: 
                        con = 'a'
                        const = 0
                        self.N2 = 1+self.N2  
                        
                    # create body
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.radius2 , self.particle_height ,self.particle_density,True,True)
                    
      
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.particle_material)
                    gran.SetName('gran'+str(con)+str(count))
                    gran.SetId(i)
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    count=count+1
                    # alternate colors on rings so one is red the other is green
                    if i%2==0:
                    # add color
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(1, 0, 0))
                        gran.AddAsset(col_r)
                    else:
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(0, 1, 0))
                        gran.AddAsset(col_r)  
                    
                    # set speed limit ( Helps but not always needed)
                    # gran.SetMaxSpeed(2)
                    # gran.SetLimitSpeed(True)
                    
                    # link to plane    
                    # pt=chrono.ChLinkMatePlane()
                    # pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    # self.my_system.AddLink(pt)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
                    
        np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)     
        self.parameters['Area'] = self.Area
        self.parameters['N1(smaller)'] = self.N1
        self.parameters['N2(larger)'] = self.N2
        
        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))             












        #### bi_dispersion_ring #####
        if self.interior_mode=="bi_dispersion_ring":  
            count=0
            for i in range(self.n.size):

                self.radius2 = self.particle_width/2 - self.offset_radius  
                
                R2=self.radius2*self.n[i]/(np.pi) 
                
                for j in range(self.n[i]):
                    radii1=self.radius2*.25
                    radii2=0
                    r_m = np.random.choice([radii1,radii2])
                    if r_m==radii1:
                        color=self.col_r
                        con='a'
                        self.N1=1+self.N1
                    else:
                        color=self.col_g
                        con='b'
                        self.N2=1+self.N2
                        
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    
                    count=count+1
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    self.Rm.append(self.radius2 - r_m)
                    self.Area = self.Area + (np.pi)*(self.radius2 - r_m)**2
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2 - r_m, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("gran"+con+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(color)
                    
                    # mate to floor
                    #pt=chrono.ChLinkMatePlane() 
                    #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    #self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)     
            self.parameters['Area'] = self.Area
            self.parameters['N1(smaller)'] = self.N1
            self.parameters['N2(larger)'] = self.N2
        
        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))
        
        

        
        
        #### bi_dispersion_uniform_ring #####
        if self.interior_mode=="bi_dispersion_uniform_ring":  
            count=0
            for i in range(self.n.size):

                self.radius2 = self.particle_width/2 - self.offset_radius  
                
                R2=self.radius2*self.n[i]/(np.pi) 
                
                for j in range(self.n[i]):
                    radii1=self.radius2
                    radii2=0
                    r_m =(1-radii1) + np.random.uniform(0, radii1)
                    if r_m==radii1:
                        color=self.col_r
                        con='a'
                    else:
                        color=self.col_g
                        con='b'
                        
                        
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    
                    count=count+1
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    self.Rm.append(self.radius2*r_m)
                    
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2*r_m, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("gran"+con+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(color)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)   
        


    def Material(self):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(self.lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
        
        
        
        
    def MaxValues(self):
        if self.interior_mode=='empty':
            N=[]
            R=0

        radius=self.bot_width/2
        radius2=self.particle_width/2 
        
        if self.interior_mode=="bidispersion":            
            Rin=self.R-radius
            S=2*radius2+2*np.sqrt(2)*radius2
            P=int(Rin/S)
            Ri=np.zeros(2*P)

            N=[]
            for i in range(P):
                Ri[2*i]=Rin-np.sqrt(2)*radius2-i*S
                Ri[2*i+1]=Rin-(2*np.sqrt(2)*radius2+radius2)-i*S
    

            for i in range(P):
                N.append(int((np.pi*2*Ri[2*i])/(2*np.sqrt(2)*radius2)))
                N.append(int((np.pi*2*Ri[2*i+1])/(2*radius2)))

        if self.interior_mode=="monodispersion":
            Rin=self.R-radius-radius2
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]
            
        if self.interior_mode=="bi_dispersion_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]           
           
            
        if self.interior_mode=="bi_dispersion_uniform_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]                  
        return(N,Ri.flatten())
    
    
    
    
    
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.particles)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.total_particles):
            self.particle_xposition["particle_xposition"+str(i)].append(self.particles[i].GetPos().x)
            self.particle_yposition["particle_yposition"+str(i)].append(self.particles[i].GetPos().y)
            self.particle_zposition["particle_zposition"+str(i)].append(self.particles[i].GetPos().z)
            

            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.total_particles):
            self.particle_xvelocity["particle_xvelocity"+str(i)].append(self.particles[i].GetPos_dt().x)
            self.particle_yvelocity["particle_yvelocity"+str(i)].append(self.particles[i].GetPos_dt().y)
            self.particle_zvelocity["particle_zvelocity"+str(i)].append(self.particles[i].GetPos_dt().z)
            
         

               
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.particle_xposition,self.particle_yposition,self.particle_zposition)
    

      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity)
        
  
class floor:
    def __init__(self,name,my_system,path):
        self.name=name
        self.my_system = my_system
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.floor_height = self.parameters['floor_height']
        self.floor_length = self.parameters['floor_length']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        
        self.material1 = self.Material(.01)
        self.material2 = self.Material(0)
        
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, -self.floor_height, 0 ))
        self.body_floor.SetMaterialSurface(self.material1)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor.GetCollisionModel().BuildModel()       
        self.body_floor.SetCollide(True)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        self.body_floor.GetAssets().push_back(body_floor_shape)
        col_k = chrono.ChColorAsset()
        col_k.SetColor(chrono.ChColor(0, 0, 0))
        self.body_floor.AddAsset(col_k)
        self.my_system.Add(self.body_floor)
        
 

        self.body_floor2 = chrono.ChBody()
        self.body_floor2.SetName('floor2')
        self.body_floor2.SetBodyFixed(True)
        self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height, 0 ))
        self.body_floor2.SetMaterialSurface(self.material2)
        self.body_floor2.GetCollisionModel().ClearModel()
        self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor2.GetCollisionModel().BuildModel()       
        self.body_floor2.SetCollide(True)
        self.my_system.Add(self.body_floor2)
        
        #body_floor_shape2 = chrono.ChBoxShape()
        #body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        #self.body_floor2.GetAssets().push_back(body_floor_shape2)
        #col_k = chrono.ChColorAsset()
        #col_k.SetColor(chrono.ChColor(0, 0, 0))
        #self.body_floor2.AddAsset(col_k)
        # self.my_system.Add(self.body_floor2)        
        # self.body_floor = chrono.ChBody()
        #self.body_floor.SetName('floor2')
        # self.body_floor.SetBodyFixed(True)
        # self.body_floor.SetPos(chrono.ChVectorD(0, self.tall+self.height2, 0 ))
        # self.body_floor.SetMaterialSurface(self.material2)
        # self.body_floor.GetCollisionModel().ClearModel()
        # self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        # self.body_floor_shape = chrono.ChBoxShape()
        
        
        # self.body_floor2 = chrono.ChBody()
        # self.body_floor2.SetName('floor2')
        # self.body_floor2.SetBodyFixed(True)
        # self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height, 0 ))
        # self.body_floor2.SetMaterialSurface(self.material2)
        # self.body_floor2.GetCollisionModel().ClearModel()
        # self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height+self.bot_height, self.floor_length) # hemi sizes
        # self.body_floor2.SetCollide(True)
        # self.my_system.Add(self.body_floor2) 
        
        #self.body_floor2_shape = chrono.ChBoxShape()
        # self.body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD((self.floor_length, self.floor_tall, self.floor_length))
        # self.body_floor2.GetAssets().push_back(self.body_floor_shape2)
  
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.length, self.tall, self.length)
        # self.body_floor.GetAssets().push_back(body_floor_shape)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor.AddAsset(col_g)body_floor.GetCollisionModel().BuildModel()       
  
    
    
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)    
    
    
    def return_enviroment(self):
        return(self.my_system)
    
    
    
class simulate:
    def __init__(self,name,my_system,bots,particles,controller,my_rep,path):
       
        self.name=name
        self.my_system = my_system
        self.Bots = bots
        self.particles = particles
        self.controller = controller
        self.my_rep = my_rep
        
        ###### Imported Variables #########
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.control_mode=self.parameters['control_mode']
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        self.visual = self.parameters['visual']
        self.dt = self.parameters['dt']
        self.time_end = self.parameters['time_end']
        self.save_rate = self.parameters['save_rate']
        
        ###### Predefined variables ######
        self.myapplication=[]
        self.epoch = 0
        self.camx = 0
        self.camy = 2
        self.camz = 0
        self.camy_height = 2
        self.save_video = False
        self.Trip=False
        self.sim_start=timeit.default_timer()
        
        ###### Empty Arrays ######
        self.time = [] # time empty array
        self.time_contact = [] # contact time  empty array
        self.number_contacts = []
        self.Contact_points_x = []
        self.Contact_points_y = []
        self.Contact_points_z = []
        self.Contact_force_x = []
        self.Contact_force_y = []
        self.Contact_force_z = []
        self.bodiesA = []
        self.bodiesB = []
        self.bodiesA_ID = []
        self.bodiesB_ID = []
        
        
        
        
        
    # simulate the robot
    def simulate(self):
        #### Irrrlecnt
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.name , chronoirr.dimension2du(800,600))
            self.myapplication.AddTypicalSky()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalCamera(chronoirr.vector3df(self.camx,self.camy,self.camz),chronoirr.vector3df(self.camx,0,self.camz))
            self.myapplication.SetSymbolscale(.002)
            self.myapplication.SetShowInfos(True)
            #self.myapplication.SetContactsDrawMode(2)
            self.myapplication.SetPaused(self.Trip)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.dt)
            self.myapplication.SetTryRealtime(False)
            ##### Run the sim
            while(self.myapplication.GetDevice().run()):
                #self.my_rep.ResetList()
                self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                self.controller.run_controller()
                self.controller.get_position()

                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.myapplication.EndScene()
                self.save_parameters()
                self.epoch = self.epoch + 1

                
                aaa=len(self.Bots.bots)
                cam_x=0.33*(self.Bots.bots[0].GetPos().x + self.Bots.bots[int(aaa/3)].GetPos().x + self.Bots.bots[int(2*aaa/3)].GetPos().x)
                cam_y=0.33*(self.Bots.bots[0].GetPos().y + self.Bots.bots[int(aaa/3)].GetPos().y + self.Bots.bots[int(2*aaa/3)].GetPos().y)
                cam_z=0.33*(self.Bots.bots[0].GetPos().z + self.Bots.bots[int(aaa/3)].GetPos().z + self.Bots.bots[int(2*aaa/3)].GetPos().z)
                self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+self.camy_height,cam_z))
                self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                self.myapplication.SetVideoframeSave(self.save_video)
                self.myapplication.SetVideoframeSaveInterval(round(1/(self.dt*60)))
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.time_end :
                    self.myapplication.GetDevice().closeDevice()
            self.sim_end=timeit.default_timer()
            
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']
            
            
        #### pov   
        if self.visual=="pov": 
            while (self.my_system.GetChTime() < self.time_end): 
                self.my_rep.ResetList()
                self.my_system.DoStepDynamics(self.dt)
                self.controller.run_controller()
                self.controller.get_position()
                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.save_parameters()
                self.epoch = self.epoch + 1
            self.sim_end=timeit.default_timer()     
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']

        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+1:
        #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        else:
            print('not save')
        
        
        with open(self.mainDirectory+self.name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))

        
    def save_parameters(self):
        ''' Function that collects data of for the system '''
        if self.epoch%self.save_rate==0:
            self.time.append(np.round(self.my_system.GetChTime(),4))
            self.Bots.save_data_position()
            self.Bots.save_data_Forces()
            self.Bots.save_data_velocity()
            self.particles.save_data_position()
            self.particles.save_data_velocity()
            self.controller.save_field_value()
            self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
            crt_list = self.my_rep.GetList()
            self.number_contacts.append(self.my_system.GetContactContainer().GetNcontacts())
            if self.my_system.GetContactContainer().GetNcontacts()!=0:
                        self.time_contact.append(self.my_system.GetChTime())
                        self.Contact_points_x.append(crt_list[0])
                        self.Contact_points_y.append(crt_list[1])
                        self.Contact_points_z.append(crt_list[2])
                        self.Contact_force_x.append(crt_list[3])
                        self.Contact_force_y.append(crt_list[4])
                        self.Contact_force_z.append(crt_list[5])
                        self.bodiesA.append(crt_list[6])
                        self.bodiesB.append(crt_list[7])
                        self.bodiesA_ID.append(crt_list[8])
                        self.bodiesB_ID.append(crt_list[9])  



                   
class controller():
    """ Class for storung all the controller information"""
    def __init__(self,name,my_system,bots,Psi,path):
        self.name=name # name of simulation
        self.my_system = my_system # the system object
        self.robots = bots # robot objets
        self.Psi=Psi # potential fields 
        
        
        ##### Extract variables from other imported objects #####
        self.forces=self.robots.force
        self.nb=self.robots.nb 
        
        ###### Imported Variables #########
        self.mainDirectory = path   # main directory 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()                 
        self.control_mode=self.parameters['control_mode']  # control mode 
        self.alpha = self.parameters['alpha']
        self.beta = self.parameters['beta']
        
        self.Field_value={}
        
        for i in range(self.nb):
            self.Field_value["bot{0}".format(i)]=[]   
            
            
        self.bot_position_x=0
        self.bot_position_z=0
        
        self.bot_velocitiy_x=0
        self.bot_velocitiy_z=0
        
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
        
    def run_controller(self):
        """Function to run the controllers"""
        if self.control_mode =="shape_formation":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.shape_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force
                
        if self.control_mode =="shape_morphing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.morph_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force            
   
            
    def save_field_value(self):
        
        if self.control_mode =="shape_formation":  
            for i in range(self.nb):
               self.Field_value["bot"+str(i)].append(self.Psi.F(self.bot_position_x[i],self.bot_position_z[i]))
        if self.control_mode =="shape_morphing":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.F_morph(self.bot_position_x[i],self.bot_position_z[i],self.Psi.tanh(t)))    
    
    
    
    def shape_controller(self):
        """ Shape controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX(self.bot_position_x[i],self.bot_position_z[i])
            Fz=self.Psi.FY(self.bot_position_x[i],self.bot_position_z[i])
            mag=np.sqrt(Fx**2+Fz**2)
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
            
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        


    def morph_controller(self):
        """ Morphing Controller for morphing from one shape to another """
        FX=[]
        FZ=[]
        t=np.round(self.my_system.GetChTime(),3)
        ft=self.Psi.tanh(t)
        #print("ft="+str(np.round(ft,2)),end='\r')
        for i in range(self.nb):
            Fx=self.Psi.FX_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            Fz=self.Psi.FY_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            
            mag=np.sqrt(Fx**2+Fz**2)
            
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
                
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
        
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        



    def get_position(self):
        """ get position of boundary robots """
        xb=[]        
        zb=[]
        for i in range(self.nb):
            xb.append(self.robots.bots[i].GetPos().x)
            zb.append(self.robots.bots[i].GetPos().z)
        return(xb,zb)
      
    def get_velocity(self):
        """ get velocity of boundary robots """
        xbv=[]
        zbv=[]
        for i in range(self.nb):
            xbv.append(self.robots.bots[i].GetPos_dt().x)
            zbv.append(self.robots.bots[i].GetPos_dt().z)
        return(xbv,zbv)

    def apply_force(self,FX,FZ):  
        """ Appy forces to robots """

        for i in range(self.nb):
            self.fxt.append(FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
            
    def clear_temp_forces(self):
        """ Clear Temp forces """
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
  



class export_data():
    def __init__(self,my_system,robots,controls,interior,simulation,Psi,my_rep,path,name):
        self.name = name # name of simulation
        self.mainDirectory = path   # main directory 
        
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.my_system = my_system
        self.robots = robots
        self.controls = controls
        self.interior = interior
        self.simulation = simulation
        
        self.results_dir=self.mainDirectory+self.name+'/results'
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)
        
        # time
        self.time = {'time':self.simulation.time} 
        
        # robot data
        (self.bot_xposition,self.bot_yposition,self.bot_zposition) = self.robots.return_position_data()
        (self.skin_xposition,self.skin_yposition,self.skin_zposition) = self.robots.return_position_membrane_data()
        (self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal) = self.robots.return_force_data()
        (self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity) = self.robots.return_velocity_data()
        (self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact) = self.robots.return_force_data_contact()            

        # interior data
        (self.particle_xposition,self.particle_yposition,self.particle_zposition) = self.interior.return_position_data()
        (self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity) = self.interior.return_velocity_data()
        
        self.Field_value=self.controls.Field_value

    def export_data(self):  
        '''Export Bot positions '''
        file_name=self.results_dir+'/bot_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xposition.items():
                w.writerow([key, *val])
            
        # write y position to csv file    
            for key, val in self.bot_yposition.items():
                w.writerow([key, *val]) 
            
        # write z position to csv file     
            for key, val in self.bot_zposition.items():
                w.writerow([key, *val])         
        
        file_name=self.results_dir+'/field_values.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.Field_value.items():
                w.writerow([key, *val])            
                
                
                
        '''Export membrane positions '''
        file_name=self.results_dir+'/membrane_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_xposition.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_yposition.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_zposition.items():
                w.writerow([key, *val])    

                
             
        ''' Export Bot velocities '''        
        file_name=self.results_dir+'/bot_velocity.csv'
        # export bot position
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xvelocity.items():
                w.writerow([key, *val])
            
        # write y position to csv file    
            for key, val in self.bot_yvelocity.items():
                w.writerow([key, *val]) 
            
        # write z position to csv file     
            for key, val in self.bot_zvelocity.items():
                w.writerow([key, *val])                   
        
        
        '''Export particle positions '''       
        file_name=self.results_dir+'/particle_position.csv'

        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xposition.items():
                w.writerow([key, *val])
            
        # write y position to csv file    
            for key, val in self.particle_yposition.items():
                w.writerow([key, *val]) 
            
        # write z position to csv file     
            for key, val in self.particle_zposition.items():
                w.writerow([key, *val])                 
                
class MyReportContactCallback(chrono.ReportContactCallback):
    """ Class for reporting and storing the the contact forces and postions  """
    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]

    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        
        IDA = bodyUpA.GetId()
        IDB = bodyUpB.GetId()
       
        self.Contact_points_x.append(vA.x)
        self.Contact_points_y.append(vA.y)
        self.Contact_points_z.append(vA.z)
        
        self.Contact_force_x.append(force.x)
        self.Contact_force_y.append(force.y)
        self.Contact_force_z.append(force.z)
        
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        
        self.bodiesA_ID.append(IDA)
        self.bodiesB_ID.append(IDB)
        

        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]

    # Get the points
    def GetList(self):
        return (self.Contact_points_x,
                self.Contact_points_y,
                self.Contact_points_z,
                self.Contact_force_x,
                self.Contact_force_y,
                self.Contact_force_z,
                self.bodiesA,
                self.bodiesB,
                self.bodiesA_ID,
                self.bodiesB_ID)




class R_functions():  
    """ R-function Class """
    def __init__(self,name):
        self.direct = os.path.dirname(__file__)
        self.name = name
        ###### Imported Variables #########
        self.mainDirectory = self.direct+"/Experiments/"
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.control_mode=self.parameters['control_mode']  # control mode
        self.m = 1
        ######### SHAPE FORMATION #########
        if self.control_mode=="shape_formation":
            self.geometry = self.parameters['geometry'] 

            if self.geometry=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['a']
            
            if self.geometry=='pacman':
                self.a = self.parameters['xcenter']
                self.b = self.parameters['zcenter']
                self.R = self.parameters['a']
                self.segments = np.array([[self.a,self.b,self.R*np.cos(np.pi/4),self.R*np.sin(np.pi/4)],[self.a,self.b,self.R*np.cos(-np.pi/4),self.R*np.sin(-np.pi/4)]])
                theta=np.linspace(np.pi/4,7*np.pi/4,100)
                self.xp=[]
                self.yp=[]
                for i in range(len(theta)):
                    
                    self.xp.append(self.R*np.cos(theta[i])+self.a)
                    self.yp.append(self.R*np.sin(theta[i])+self.b)
                
                self.xp.append(self.R*np.cos(-np.pi/4))
                self.xp.append(self.a)
                self.xp.append(self.R*np.cos(np.pi/4))
                
            
                self.yp.append(self.R*np.sin(-np.pi/4))
                self.yp.append(self.a)
                self.yp.append(self.R*np.sin(np.pi/4))                
                
                
                np.savez(self.mainDirectory+'/'+self.name+'/'+'outline'+self.geometry+'.npz',xp=self.xp,yp=self.yp)          
            else:
                data = np.load(self.direct+'/shapes/'+self.geometry+'.npz')
                self.segments = data['segments']
                self.scale = self.parameters['scale']
                self.segments=self.segments
                self.xp=data['xp']
                self.yp=data['yp']
                np.savez(self.mainDirectory+'/'+self.name+'/'+'outline'+self.geometry+'.npz',xp=self.xp,yp=self.yp)          

            
            

            
        ######### SHAPE MORPHING #########
        if self.control_mode=="shape_morphing":  
            self.p = self.parameters['p']
            self.geometry1 = self.parameters['geometry1'] 
            self.geometry2 = self.parameters['geometry2'] 
            self.scale1 = self.parameters['scale1']
            self.scale2 = self.parameters['scale2']
            
        
            
            if self.geometry1=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']
                
                
            if self.geometry2=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']


            if self.geometry1!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry1+'.npz')
                self.segments = data['segments']
                self.segments = self.scale1*self.segments                   
                
            
            if self.geometry2!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry2+'.npz')
                self.segments = data['segments']
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes'+self.geometry2+'.npz',segments=self.segments,xp=data['xp'],yp=data['yp'])          
               
            
            
##############################################################################            
            
    def phi_circle(self,x,y,a,b,R):
        """ Normalized distance function of a circle """
        phi = (R**2 - (x-a)**2 - (y-b)**2)
        return(abs(phi))
     
        
    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def phi_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)

        T=self.phi_line_(x,y,x1,y1,x2,y2)
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        
        phi1=self.Trim(f,T)
        
        return(phi1)
    
    
    def dphix_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt x  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Tx=self.dphix_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fx=self.dphix_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimx(f,T,fx,Tx)
        
        return(phi1)    
    

    def dphiy_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt y  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Ty=self.dphiy_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fy=self.dphiy_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimy(f,T,fy,Ty)
        
        return(phi1)


    
    
    def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

    def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
    def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
    
##############################################################################   

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   


    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    
    
    
    
    def FX(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
            
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1x = self.dphix_quarter_circle(x,y,self.a,self.b,self.R)
            
            f2 = self.phi_segments(x,y,self.segments)
            f2x = self.dphix_segments(x,y,self.segments)
            
            Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def FY(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1y = self.dphiy_quarter_circle(x,y,self.a,self.b,self.R)
        
            f2 = self.phi_segments(x,y,self.segments)
            f2y = self.dphiy_segments(x,y,self.segments)
            
            Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m +f2**self.m))
            
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    
    
    
    def F(self,x,y):
       """ Single function to call field"""
       if self.geometry=='circle':
           F = self.phi_circle(x,y,self.a,self.b,self.R)
            
       if self.geometry=='pacman':
           f = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
           f2=self.phi_segments(x,y,self.segments)
           F = (f*f2)/((f**self.m +f2**self.m)**(1/self.m))
           
         
       else:
           F = self.phi_segments(x,y,self.segments)
           
       return(F)

       
##################################################################################    
    
#### MORPHING FUNCTIONS
    

    def F_morph(self,x,y,ft):
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)        
        F = self.C_morph(self,phi1,phi2,ft)
        return(F)
    
    def FX_morph(self,x,y,ft):
        """ Control for x component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1x = self.F1X(x,y)
        dphi2x = self.F2X(x,y)
        
        Fx = self.C_morphx(phi1,phi2,dphi1x,dphi2x,ft)
        
        
        return(Fx)

    def FY_morph(self,x,y,ft):
        """ Control for y component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1y = self.F1Y(x,y)
        dphi2y = self.F2Y(x,y)
        
        Fy = self.C_morphy(phi1,phi2,dphi1y,dphi2y,ft)
        
    
        return(Fy)
    
    

    def F1(self,x,y):
        """ Field of the initial starting field """
        if self.geometry1=='circle':
            F1 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F1 = self.dphix_segments(x,y,self.segments1)
    
        return(F1)        
        
    def F2(self,x,y):
        """ Field of the desired  field """
        if self.geometry2=='circle':
            F2 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F2 = self.phi_segments(x,y,self.segments)
    
        return(F2)      


    def F1X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry1=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F1Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry1=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    


    def F2X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry2=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F2Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry2=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)  


    def tanh(self,t):
        """ tanh function """
        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        #print('tanh=',tanh)
        return(tanh)   
    
    
    def g1(self,phi1,t):
        """ intersection of initial field and -f(t) """
        return(phi1 - t - np.sqrt(phi1**2 + t**2))
        
    def g2(self,phi2,t):
        """ intersection of final field and f(t)-1 """
        return(phi2 + (t-1) - np.sqrt(phi2**2+(t-1)**2))
    
    
    def dgx(self,phi,dphix,s):
        """ derivative of g1 or g2 wrt x """
        # s is either (t) or t-1
        return(dphix-(phi*dphix)/(np.sqrt(s**2 +phi**2)))
    
    
    def dgy(self,phi,dphiy,s):
        """ derivative of g1 or g2 wrt y """
        return(dphiy-(phi*dphiy)/(np.sqrt(s**2 +phi**2)))    
    
    
    def w1(self,g1,g2): 
        """ Weighted function 1 """
        return(g2/(g1+g2))
    
    
    def w2(self,g1,g2):
        """ Weighted function 2 """
        return(g1/(g1+g2))    
    
  
    def dw1x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 1 wrt x """
        return((dg2x) / (g1+g2) - (dg1x+dg2x)*g2 / (g1+g2)**2)
    
    
    def dw2x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 2 wrt x """
        return((dg1x) / (g1+g2) - (dg1x+dg2x)*g1 / (g1+g2)**2)
    
    
    def dw1y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 1 wrt y """
        return((dg2y) / (g1+g2) - (dg1y+dg2y)*g2 / (g1+g2)**2)
    
    
    def dw2y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 2 wrt y """
        return((dg1y) / (g1+g2) - (dg1y+dg2y)*g1 / (g1+g2)**2)  
    
  

    def C_morph(self,phi1,phi2,ft):
        """ Morphing function from phi1 to phi2 """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        return(W1*phi1+W2*phi2)
    
    

    def C_morphx(self,phi1,phi2,dphi1x,dphi2x,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt x """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1x = self.dgx(phi1,dphi1x,ft)
        dg2x = self.dgx(phi2,dphi2x,ft-1)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        DW1X = self.dw1x(dg1x,dg2x,G1,G2)
        DW2X = self.dw2x(dg1x,dg2x,G1,G2)
        
        return(phi1*DW1X + phi2*DW2X + W1*dphi1x + W2*dphi2x)
    
    def C_morphy(self,phi1,phi2,dphi1y,dphi2y,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt y """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1y = self.dgy(phi1,dphi1y,ft)
        dg2y = self.dgy(phi2,dphi2y,ft-1)
    
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        DW1Y = self.dw1y(dg1y,dg2y,G1,G2)
        DW2Y = self.dw2y(dg1y,dg2y,G1,G2)
        
        return(phi1*DW1Y + phi2*DW2Y + W1*dphi1y + W2*dphi2y)




    def create_segment(self,x,y):
        """ Create segment matrix for points of R-function """
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)        
 
    
    def plot_zero_contours(self,xp1,yp1,xp2,yp2,d):
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)         
        axs.plot(xp1,yp1,color='k',linewidth=3)
        axs.plot(xp2,yp2,color='tab:red',linewidth=3)
        
        
    def plot_R_function(self,X,Y,R,Rx,Ry,d):
        """ Plot R-function and its derivatives """
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(10,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
        
        # Plot Phi
        im1=axs[0].contourf(X, Y,R,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        axs[0].contour(X,Y,R,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        #axs[0].plot(xp,yp,color='k',linewidth=3)
        axs[0].set_title('$\phi(x)$')
        axs[0].set_xticks(xticks)
        axs[0].set_yticks(xticks)
        fig.colorbar(im1, ax=axs[0])
        
        # Plot Phix
        im2=axs[1].contourf(X, Y,Rx,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[1].plot(xp,yp,color='k',linewidth=3)
        axs[1].contour(X,Y,Rx,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[1].set_xticks(xticks)
        axs[1].set_yticks(xticks)
        axs[1].set_title(r"$\frac{\partial \phi}{\partial x}$")
        fig.colorbar(im2, ax=axs[1])
        
        # Plot Phiy
        im3=axs[2].contourf(X,Y,Ry,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[2].plot(xp,yp,color='k',linewidth=3)
        axs[2].contour(X,Y,Ry,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[2].set_xticks(xticks)
        axs[2].set_yticks(xticks)
        axs[2].set_title(r"$\frac{\partial \phi}{\partial y}$")
        fig.colorbar(im3, ax=axs[2])
         
        
        
    def plot_R_function_morph(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1y,dphi2y,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks
        for i in range(nr):
            for j in range(nc):
                #print(np.min(C[count]))
                CS1 = axs[i,j].contour(X,Y,CX[count],levels = [0],colors=('tab:red'),linestyles=('-',),linewidths=(3,))
                CS2 = axs[i,j].contour(X,Y,CY[count],levels = [0],colors=('tab:blue'),linestyles=('--',),linewidths=(3,))
                #(xi,yi)=find_intersection(CS1,CS2)
                #axs[i,j].plot(xi,yi,'ko', ms=3)
                #CS2 = axs[i,j].contour(X,Y,CX[count],5,colors='black',zorder=0)
                #axs[i,j].set_xlim([-d,d])
                #axs[i,j].set_ylim([-d,d])
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
        fig.tight_layout()
        
        
        
    def plot_R_function_morph_color(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1x,dphi2x,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks        
        for i in range(nr):
            for j in range(nc):
                print(np.min(CX[count]))
                #axs[i,j].axis('equal')
                #CS1 = axs[i,j].contour(X,Y,C[count],linewidths=(1,))
                im1=axs[i,j].contourf(X, Y,C[count],cmap = 'jet',levels=50,alpha=1,linestyles='solid')  
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
                fig.colorbar(im1, ax=axs[i,j])
        plt.tight_layout()        



def plot_line(xp,yp):  
    fm._rebuild()
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'dejavuserif'
    plt.rcParams['font.size'] = 6
    plt.rcParams['axes.linewidth'] = .1
    #xticks = np.linspace(-d, d,5,endpoint=True)
    fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(10,3))
    fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
    axs.plot(xp,yp,linewidth=1,color='k')
        
        
def shoelace(vertices):
    """ Find the cross sectional area"""
    (m,n)=np.shape(vertices)

    sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
    for i in range(1,n-1):
        sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
    i=n-1
    sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

    A=.5*abs(sum1)
    return (A)            


def create_segment(x,y):
    """ Create segment matrix for points of R-function """
    seglen=len(x)
    segments=np.zeros((seglen-1,4))
    for i in range(seglen-1):
        #[x1,y1,x2,y2]
        #[x2,y2,x3,y3]
        segments[i,0]=x[i]
        segments[i,1]=y[i]
        segments[i,2]=x[i+1]
        segments[i,3]=y[i+1]
    return(segments)     





class import_data:
     def __init__(self,name,path,wxmin,wxmax,wymin,wymax):
         self.name=name
         self.path=path
         self.mainDirectory = path   # main directory 
         parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
         data=np.load(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',allow_pickle=True) 
         self.Rm=data['Rm'] 
         self.wxmin = wxmin
         self.wxmax = wxmax
         self.wymin = wymin
         self.wymax = wymax
         
         self.parameters=parameters.tolist()           
         self.nb=self.parameters['nb'] # number of bots
         self.ni=self.parameters['total_particles']
         self.ns=self.parameters['ns']
         self.nm=self.nb*self.ns
         self.bot_width=self.parameters['bot_width']
         self.particle_width=self.parameters['particle_width']
         self.control_mode=self.parameters['control_mode']
         self.skin_width=self.parameters['skin_width']
         
         if self.control_mode=="shape_formation":
             self.geometry = self.parameters['geometry']
             data2=np.load(self.mainDirectory+self.name+'/outline'+self.geometry+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
                  
         if self.control_mode=="shape_morphing":
             self.geometry1=self.parameters['geometry1']
             self.geometry2=self.parameters['geometry2']
             data2=np.load(self.mainDirectory+self.name+'/shapes'+self.geometry2+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
         
            
         
         self.path=self.path+self.name+"/results/"
         os.chdir(self.path)
         self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
         
         
         # Robot Position
         self.bot_position=np.genfromtxt(self.files[self.files.index('bot_position.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_position)
         self.bot_position=self.bot_position[:,1:self.n1]
         self.time=self.bot_position[0,:]
         self.bot_position_x=self.bot_position[1:self.nb+1,:]
         self.bot_position_y=self.bot_position[self.nb+1:2*self.nb+1,:]
         self.bot_position_z=self.bot_position[(2*self.nb)+1:3*self.nb+1,:] 
         
         # membrane_positions
         self.membrane_position=np.genfromtxt(self.files[self.files.index('membrane_position.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.membrane_position)
         self.membrane_position=self.membrane_position[:,1:n]
         self.membrane_position_x=self.membrane_position[1:self.nm+1,:]
         self.membrane_position_y=self.membrane_position[self.nm+1:2*self.nm+1,:]
         self.membrane_position_z=self.membrane_position[(2*self.nm)+1:3*self.nm+1,:]          
         
         self.Field_value=np.genfromtxt(self.files[self.files.index('field_values.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.Field_value)
         self.Field_value=self.Field_value[:,1:n]
         
         self.Field_value_sum=[]
         
         for i in range(len(self.time)):
             self.Field_value_sum.append(np.sum(abs(self.Field_value[:,i])))
         
         # Particle Position
         self.particle_position=np.genfromtxt(self.files[self.files.index('particle_position.csv') ] ,delimiter=',')
         (self.m4a,self.n4a)=np.shape(self.particle_position)
         self.particle_position=self.particle_position[:,1:self.n4a]
         self.particle_position_x=self.particle_position[1:self.ni+1,:]
         self.particle_position_y=self.particle_position[self.ni+1:2*self.ni+1,:]
         self.particle_position_z=self.particle_position[(2*self.ni)+1:3*self.ni+1,:]
         
     def create_frames(self,membrane):
        ''' Create frames for a video '''
        direct = os.path.join(self.mainDirectory+self.name,'_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(4, 4)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))

            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='black')
                ax.add_patch(patch)
            
                
            
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    ax.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                ax.add_patch(patch)         
         
            ax.plot(self.xp,self.yp,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            plt.title('Time= ' + str(np.round(self.time[i],3)),fontsize=8)
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          

     def plot_field_values(self):
         import matplotlib.font_manager as fm
         # Rebuild the matplotlib font cache
         fm._rebuild()
         plt.rcParams['font.family'] = 'Times New Roman'
         plt.rcParams['mathtext.fontset'] = 'dejavuserif'
         plt.rcParams['font.size'] = 6
         plt.rcParams['axes.linewidth'] = .1
         plt.rcParams["text.usetex"] = True
         fig, ax = plt.subplots(figsize=(2, 2),dpi=300)


                
         ax.plot(self.time,self.Field_value_sum,color='red',linewidth=1)
        
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.Field_value_sum), np.max(self.Field_value_sum),5,endpoint=True)
         ax.set_xticks(np.round(x_ticks,2))
         ax.set_yticks(np.round(y_ticks,2))
         ax.xaxis.set_tick_params(width=.25,length=2)
         ax.yaxis.set_tick_params(width=.25,length=2)
         ax.set_ylabel(r"$\Sigma \phi$")
         ax.set_xlabel("Time (seconds)")
         ax.set_title("Field Values")
         ax.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.pdf')        


     def create_snap_shot(self,entry,membrane):
        ''' Create snapshots  '''
        i=entry
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 6
        plt.rcParams['axes.linewidth'] = .1
        plt.rcParams["text.usetex"] = True 
        fig, ax = plt.subplots(figsize=(1.5, 1.5),dpi=300)
        #plt.tight_layout()
        #fig.subplots_adjust(top=0.94,
        #                    bottom=0.191,
        #                    left=0.21,
        #                    right=0.919,
        #                    hspace=0.2,
        #                    wspace=0.2)
        #plt.axis('off')
        #ax.axis('equal')
        #ax.axis('equal')
        #plt.autoscale(enable=True, axis='y')
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            patch = plt.Circle((x0, y0),self.bot_width/2, fc='black')
            ax.add_patch(patch)
            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if self.Rm[j]==self.particle_width/2:
                c='tab:blue'
            if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
            ax.add_patch(patch)         
     
        ax.plot(self.xp,self.yp,color='tab:red',linestyle='dashed',linewidth=1.0,zorder=0)
        #plt.title('Time = ' + str(np.round(self.time[i],2)),fontsize=6)
        
        xticks = np.linspace(self.wxmin, self.wxmax,3,endpoint=True)
        yticks = np.linspace(self.wymin, self.wymax,3,endpoint=True)
        
        
        ax.set_xticks(np.round(xticks,2))
        ax.set_yticks(np.round(yticks,2))
        ax.xaxis.set_tick_params(width=.25,length=2)
        ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel("$y$ (meters)")
        ax.set_xlabel("$x$ (meters)")
        #ax.axis('equal')
        plt.tight_layout()
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.jpg')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.svg')    
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.pdf')
    
       # plt.close('all')   
            
   
     def create_video(self):
         #import pdb    
         img_array = []
         for index, filename in enumerate(glob.glob(self.mainDirectory+'/'+self.name+'/_frames/'+'/*.jpg')):
             #pdb.set_trace()
             img = cv2.imread(filename)
             height, width, layers = img.shape
             size = (width,height)
             img_array.append(img)
         out = cv2.VideoWriter(self.mainDirectory+'/'+self.name+'/video.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (width,height))    

         for i in range(len(img_array)):
            out.write(img_array[i])
         out.release()          
         
     # def create_video(self):         
     #    nsteps=len(self.time)
     #    tend=self.time[-2]
     #    FPS = 60
     #    #framesNum = int(FPS*tend)
     #    framesNum=len(self.time)
     #    fig = plt.figure(figsize=(6,6))
     #    #fig.set_size_inches(10, 10)
    
     #    ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
     #    bots=[]
     #    for i in range(self.nb):
     #        x,y = self.bot_position_x[i,0],self.bot_position_z[i,0]
     #        patch = plt.Circle((x, y), self.bot_width/2, fc='k')
     #        bots.append(patch)
    
        
     #    def init():
     #        ax.grid(True)
     #        for i in range (0,len(bots)):
     #            ax.add_patch(bots[i])
                
     #        return []
    
     #    def animatePatches(i,bots):
     #        for j in range (0,len(bots)):
     #            x,y = self.bot_position_x[j,i],self.bot_position_z[j,i]
     #            bots[j].center=(x, y)
                
    
     #        return bots,
    
    
     #    def animationManage(i,bots):
     #        animatePatches(i,bots)
     #        plt.title('time= ' + str(np.round(self.time[i],3)))
     #        return []
    
     #    anim = animation.FuncAnimation(fig, animationManage,init_func=init,interval=100,fargs=(bots,))

     #    #plt.show()
     #    #return(HTML(anim.to_html5_video()))    