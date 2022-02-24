# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 16:06:53 2022

@author: dmulr
"""


import pybullet as p
import pybullet_data
from numpy import pi
import numpy as np
import pathlib
import sys
import pywavefront
from poisson_sampling import *
from get_user import *
from RFunctions import *
import os

file_path = pathlib.Path(__file__).parent.resolve()

from QuaternionRotation import create_from_axis_angle
import pdb

from tqdm import tqdm
get_indexes = lambda x, xs: [i for (y, i) in zip(xs, range(len(xs))) if x == y]
if __name__ =='__main__':
    #physicsClient = p.connect(p.GUI)
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data_dict[user])

    path = os.path.dirname(__file__)
    path = path+"/Experiments/"
    os.chdir(path)
    files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
    name = files[-1]
    print(name)
    parameters = np.load(path+'\\'+name+'\\Parameters.npy',allow_pickle=True)
    parameters = parameters.tolist()
    
    
    collect_data = parameters['collect_data']
    timestep = parameters['timestep']
    tend = parameters['tend']
    save_rate = parameters['save_rate']
    end = parameters['end']
    #print(parameters['shape'])
    #print(parameters['shape_path'])
    obj_file = parameters['shape_path'] + parameters['shape']
    #print(obj_file)
    icosphere_radius = parameters['icosphere_radius']
    springElastic = parameters['springElasticStiffness']
    springDamping = parameters['springDampingStiffness']
    BendingSprings = parameters['useBendingSprings']
    mu = parameters['frictionCoeff']
    nb = parameters['nb']
    
    #### INTERIOR GEOMETRY ####
    particle_radius = parameters['particle_radius']
    particle_width = parameters['particle_width']
    particle_mass = parameters['particle_mass']
    
    xc = parameters['xc']
    yc = parameters['yc']
    zc = parameters['zc']
    
    r_ = parameters['r_']
    segs = parameters['segs']
    xsegs = parameters['xsegs']
    rad_seg = parameters['rad_seg']
    theta = parameters['theta']
    #### CONTROL SPHERES #### 
    control_sphere_mass = parameters['control_sphere_mass']
    control_sphere_radius = parameters['control_sphere_radius'] 
    
    #### CONTROL MODE ####
    force_mag = parameters['force_mag'] 
    beta = parameters['beta']
    des_rad = parameters['des_rad']
    desired_shape = parameters['desired_shape'] 

#     #obj_file = 'Blender/Icosphere2.obj'
    
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    
    p.resetDebugVisualizerCamera(1,0,-30,[0,0,0])
    rotation = create_from_axis_angle(1,0,0,0)
    
      # In[Create Plane]
    plane = p.loadURDF("plane.urdf", [0,0,-.25])
    p.createMultiBody(0, plane)
    p.setGravity(0, 0, -9.81)
    rotation = create_from_axis_angle(1,0,0,0)
    #print(shape)
      #path2="F:/Soro_chrono/python/PyBullet/3-D_strings/Blender/Icosphere2.obj"
    #print(obj_file)
    scene = pywavefront.Wavefront(obj_file)
    
#     scene = path
    faces=scene.mesh_list[0].faces
    vertices2=scene.vertices
    vertices2=np.asarray(vertices2)  
    
    


      # In[General Variables]
    
    col=0
    Time=[]
    DATA=[]
      # In[Create mesh]
      # All the parameters in this function need to be played with to understand their affects
    mesh = p.loadSoftBody(obj_file,
                              basePosition = [0,0,0],
                              baseOrientation = rotation,
                              springElasticStiffness = springElastic,
                              springDampingStiffness = springDamping,
                              # springBendingStiffness = 10,
                              scale=1,
                              mass=0.1,
                              useMassSpring=1,
                              useBendingSprings=BendingSprings,
                              # useNeoHookean = 1,
                              #useSelfCollision=1,
                              useFaceContact=True,
                              frictionCoeff=mu
#                             # repulsionStiffness=800
                              )

    # Editing visual for mesh   
    p.changeVisualShape(mesh, -1, rgbaColor=[1,.412,.706,0.5], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    # Get Mesh data
    node_loc = []
    data = p.getMeshData(mesh, -1,-1, flags=p.MESH_DATA_SIMULATION_MESH)
    #DATA.append(data)
    verts = np.asarray(data[1])
    
    if collect_data: node_loc.append(verts[0])

    # In[Interior particle creation]

    particle_positions = []
    ri=[]
    ni=[]
    for i in range(len(rad_seg)):
        Rin=rad_seg[i]-(particle_width/2)#-(self.particle_width/2)
        ngrans1=int(Rin/(particle_width))
        rit=np.zeros((1,ngrans1))
        nit=np.zeros((1,ngrans1))
        radii=Rin-(particle_width)
        for j in range(ngrans1):
            remainder=((particle_width))*j
            rit[:,j]=radii-remainder
            nit[:,j]=np.floor(((rit[:,j]*np.pi)/(particle_width/2)))
        ri.append(rit)
        ni.append(nit)
    
    
    for k in range(len(rad_seg)):
    #     particle_width
        n=np.asarray(ni[k],dtype=int)
        N=n[0]
        r=ri[k]
        Ri=r.flatten()
    
        for i in range(N.size):
            for j in range(N[i]):
                R2=(particle_width)*N[i]/np.pi
                x=xsegs[k]
                z=R2*np.cos(j*2*np.pi/N[i])
                y=R2*np.sin(j*2*np.pi/N[i])
                particle_positions.append([x,y,z])



    print('number of particles: '+str(len(particle_positions))) 
    parameters['number of particles']=len(particle_positions)
    np.save(path+'\\'+name+'\\Parameters.npy',parameters)
    in_particle = p.createCollisionShape(p.GEOM_SPHERE,radius = particle_width)
    particle_IDs = []
    for pos in particle_positions:
        particle_IDs.append(p.createMultiBody(
                            baseMass = particle_mass,
                            baseCollisionShapeIndex = in_particle,
                            basePosition = pos))    
        
    
    Particle_positions_savex=np.zeros((len(particle_positions),int(end/save_rate)))    
    Particle_positions_savey=np.zeros((len(particle_positions),int(end/save_rate)))    
    Particle_positions_savez=np.zeros((len(particle_positions),int(end/save_rate)))       
    
    
    #Force_savex=np.zeros((len(nb),int(end/save_rate)))    
    #Force_savey=np.zeros((len(nb),int(end/save_rate)))    
    #Force_savez=np.zeros((len(nb),int(end/save_rate)))       
    
    Force_savex=np.zeros((nb,int(end/save_rate)))    
    Force_savey=np.zeros((nb,int(end/save_rate)))    
    Force_savez=np.zeros((nb,int(end/save_rate)))           
    get_indexes = lambda x, xs: [i for (y, i) in zip(xs, range(len(xs))) if x == y]
# In[Create Robots]    

    control_sphere = p.createCollisionShape(p.GEOM_SPHERE,
                                radius = control_sphere_radius)
    control_sphere_visual = p.createVisualShape(p.GEOM_SPHERE,
                                        radius = control_sphere_radius,
                                        rgbaColor = [0,0,0,1])                  

   
    
    
    #control_sphere_locs = np.zeros((len(nb), 3))
    control_sphere_locs = np.zeros((nb, 3))    
    ccc=0
    for i, vert in enumerate(verts):
        #print(i)
        #print(get_indexes(i,nb))
        #print(vert)
        #print(len(get_indexes(i,nb)))
        #if len(get_indexes(i,nb))==1:
            #print(len(get_indexes(i,nb)))
        dirt = vert / np.linalg.norm(vert)
        addition = dirt*control_sphere_radius
        control_sphere_locs[ccc] = vert + addition*1.1
        ccc=ccc+1

    #### EMPTY BOT POSITIONS ####
    #bot_positions_savex=np.zeros((len(nb),int(end/save_rate)))    
    #bot_positions_savey=np.zeros((len(nb),int(end/save_rate)))    
    #bot_positions_savez=np.zeros((len(nb),int(end/save_rate)))  
    #F_value=np.zeros((len(nb),int(end/save_rate)))  

    bot_positions_savex=np.zeros((nb,int(end/save_rate)))    
    bot_positions_savey=np.zeros((nb,int(end/save_rate)))    
    bot_positions_savez=np.zeros((nb,int(end/save_rate)))  
    F_value=np.zeros((nb,int(end/save_rate)))  


    # Creating the control spheres
    control_sphere_Ids = []
    for index, pos in enumerate(control_sphere_locs):
        control_sphere_Ids.append(p.createMultiBody(
                                    baseMass = control_sphere_mass,
                                    baseInertialFramePosition = [0,0,0],
                                    baseCollisionShapeIndex = control_sphere,
                                    basePosition = pos,
                                    baseVisualShapeIndex = control_sphere_visual))
    
    # Attaching the control sphere to mesh
    for index, cs in enumerate(control_sphere_Ids):
        pass
        p.createSoftBodyAnchor(mesh, index, cs, -1)


    # In[ Simulate]
    p.setRealTimeSimulation(0)
    # timestep = 1/240.
    p.setTimeStep(timestep)    
    T = 0 
    count = 0
    for i in tqdm(range(end)):
        if not p.isConnected: 
            break

        T+=1
        if collect_data:
            if T%end==0:
                pass
                print(T)
        if T>end:
            break
        pass  
        p.stepSimulation()

        #Apply a force to control bodies in direction of origin
        cc=0
        #for index, c_ball in enumerate(control_sphere_Ids):
        for c_ball in control_sphere_Ids:
           
            pos = np.asarray(p.getBasePositionAndOrientation(c_ball)[0])
            vel = np.asarray(p.getBaseVelocity(c_ball)[0])
            #n_pos = pos / np.linalg.norm(pos)
            
            
            
            if desired_shape=="Cube":
                Fx = DF_box_x(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                Fy = DF_box_y(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                Fz = DF_box_z(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)

            
            if desired_shape=="Sphere":
                Fx = dFsphere_x(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                Fy = dFsphere_y(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                Fz = dFsphere_z(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)

            
            if desired_shape=="Pryamid":

                Fx = DF_Pryamid_x(pos[0],pos[1],pos[2],xc,yc,zc,des_rad,theta)
                Fy = DF_Pryamid_y(pos[0],pos[1],pos[2],xc,yc,zc,des_rad,theta)
                Fz = DF_Pryamid_z(pos[0],pos[1],pos[2],xc,yc,zc,des_rad,theta)
                    
                    
                    
            if desired_shape=="Sphere" or desired_shape=="Cube" or desired_shape=="Pryamid":
                F=np.sqrt(Fx**2 + Fy**2 + Fz**2)
                FX=Fx/F
                FY=Fy/F
                FZ=Fz/F
            
                n_pos=np.array([FX,FY,FZ])
                force=np.zeros(3)
                
                force[0]=-force_mag*FX-beta*vel[0]
                force[1]=-force_mag*FY-beta*vel[1]
                force[2]=-force_mag*FZ-beta*vel[2]
                #force = -force_mag*np.sin(30*count*timestep)*n_pos
                p.applyExternalForce(c_ball,-1, force, n_pos, p.WORLD_FRAME)
                if collect_data==True and count%save_rate==0:
                
                    Force_savex[cc,col]=force[0]
                    Force_savey[cc,col]=force[1]
                    Force_savez[cc,col]=force[2]
                    cc=cc+1
                    
        if collect_data==True and count%save_rate==0:
            data = p.getMeshData(mesh,-1,flags=p.MESH_DATA_SIMULATION_MESH)
            DATA.append(data)
            num=0
            for cball in control_sphere_Ids:
                pos = np.asarray(p.getBasePositionAndOrientation(cball)[0])
                if desired_shape=="Sphere":
                    F_value[num,col] = Fsphere(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                if desired_shape=="Cube":
                    F_value[num,col] = F_box(pos[0],pos[1],pos[2],xc,yc,zc,des_rad)
                
                if desired_shape=="Pryamid":
                    F_value[num,col] = F_Pryamid(pos[0],pos[1],pos[2],xc,yc,zc,des_rad,theta)                    
                
                bot_positions_savex[num,col]=pos[0]
                bot_positions_savey[num,col]=pos[1]
                bot_positions_savez[num,col]=pos[2]
                num=num+1
                
            num=0
            for cball in particle_IDs:
                pos = np.asarray(p.getBasePositionAndOrientation(cball)[0])
                Particle_positions_savex[num,col]=pos[0]
                Particle_positions_savey[num,col]=pos[1]
                Particle_positions_savez[num,col]=pos[2]
                num=num+1
            
            col=col+1                
            Time.append(count*.001)        
        count=count+1


with open(path+'/'+name+"/Parameters.csv", 'w') as f:
    for key in parameters.keys():
        f.write("%s, %s\n" % (key, parameters[key]))




np.savez(path+'/'+name+'/'+'data.npz',
          Particle_positions_savex=Particle_positions_savex,
          Particle_positions_savey=Particle_positions_savey,
          Particle_positions_savez=Particle_positions_savez,
          bot_positions_savex=bot_positions_savex,
          bot_positions_savey=bot_positions_savey,
          bot_positions_savez=bot_positions_savez,
          Force_savex=Force_savex,
          Force_savey=Force_savey,
          Force_savez=Force_savez,
          F_value=F_value,
          Time=Time,
          DATA=DATA,
          faces=faces)
