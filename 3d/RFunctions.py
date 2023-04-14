# -*- coding: utf-8 -*-
"""
Created on Tue Jan 18 10:39:20 2022

@author: dmulroy
"""
import numpy as np
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt
import IPython
from sympy import *
import math
import scipy.constants                # For the golden ratio
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm                       # This is for the color map




def Fsphere(x,y,z,xc,yc,zc,R):
    ''' ADF fpr a sphere '''
    return(abs((x-xc)**2 + (y-yc)**2 + (z-zc)**2 - R**2))

def dFsphere_x(x,y,z,xc,yc,zc,R):
    ''' Derivative wrt x of a sphere ADF '''
    return(2*(x-xc)*np.sign((x-xc)**2 + (y-yc)**2 + (z-zc)**2 - R**2))

def dFsphere_y(x,y,z,xc,yc,zc,R):
    ''' Derivative wrt y of a sphere ADF '''
    return(2*(y-yc)*np.sign((x-xc)**2 + (y-yc)**2 + (z-zc)**2 - R**2))

def dFsphere_z(x,y,z,xc,yc,zc,R):
    ''' Derivative wrt z of a sphere ADF '''
    return(2*(z-zc)*np.sign((x-xc)**2 + (y-yc)**2 + (z-zc)**2 - R**2))


def F_plane_n(x,y,z,n,p):
    ''' Distance function for a plane '''
    v=[x,y,z]
    n=n/np.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
    d=(x-p[0])*n[0] + (y-p[1])*n[1] + (z-p[2])*n[2]
    return(d)

def dF_plane_nx(x,y,z,n,p):
    ''' Derivative wrt x of a plane ADF '''
    v=[x,y,z]
    n=n/np.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
    
    dx=n[0]
    return(dx)

def dF_plane_ny(x,y,z,n,p):
    ''' Derivative wrt y of a plane ADF '''
    v=[x,y,z]
    n=n/np.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
    
    dy=n[1]
    return(dy)

def dF_plane_nz(x,y,z,n,p):
    ''' Derivative wrt z of a plane ADF '''
    v=[x,y,z]
    n=n/np.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
    
    dz=n[2]
    return(dz)


def union(w1,w2):
    ''' R-union '''
    return(w1+w2+np.sqrt(w1**2 + w2**2))

def union_x(w1,w2,w1x,w2x):
    ''' Derivative wrt x of R-union  '''
    return((w1*w1x + w2*w2x)/np.sqrt(w1**2 + w2**2) + w1x + w2x)

def union_y(w1,w2,w1y,w2y):
    ''' Derivative wrt y of R-union  '''
    return((w1*w1y + w2*w2y)/np.sqrt(w1**2 + w2**2) + w1y + w2y)

def union_z(w1,w2,w1z,w2z):
    ''' Derivative wrt z of R-union  '''
    return((w1*w1z + w2*w2z)/np.sqrt(w1**2 + w2**2) + w1z + w2z)



def intersection(w1,w2):
    ''' R-interstion '''
    return(w1+w2-np.sqrt(w1**2 + w2**2))


def equivelence(w1,w2,m):
    ''' R-equivelence'''
    return(w1*w2/((w1**m + w2**m)**(1/m)))

    
def F_box(x,y,z,xc,yc,zc,m): 
    ''' ADF for a box '''
    n1=[0,0,1]
    p1=[0,0,1*(m + zc)]
    
    n2=[0,1,0]
    p2=[0,1*(m + yc),0]
    
    n3=[0,0,-1]
    p3=[0,0,-1*(m + zc)]    
    
    n4=[1,0,0]
    p4=[1*(m + xc),0,0]    
    
    n5=[0,-1,0]
    p5=[0,-1*(m + yc),0]
    
    n6=[-1,0,0]
    p6=[-1*(m + xc),0,0]
    
    d1=F_plane_n(x,y,z,n1,p1)
    
    d2=F_plane_n(x,y,z,n2,p2)
    d=union(d1,d2)

    
    d3=F_plane_n(x,y,z,n3,p3)  
    d=union(d,d3)    

    
    d4=F_plane_n(x,y,z,n4,p4)
    d=union(d,d4)     
    

    d5=F_plane_n(x,y,z,n5,p5)
    d=union(d,d5)     



    d6=F_plane_n(x,y,z,n6,p6)
    d=union(d,d6)    

    return(abs(d))


def DF_box_x(x,y,z,xc,yc,zc,m): 
    ''' derivative wrt x for an ADF for a box '''
    n1=[0,0,1]
    p1=[0,0,1*(m + zc)]
    
    n2=[0,1,0]
    p2=[0,1*(m + yc),0]
    
    n3=[0,0,-1]
    p3=[0,0,-1*(m + zc)]    
    
    n4=[1,0,0]
    p4=[1*(m + xc),0,0]    
    
    n5=[0,-1,0]
    p5=[0,-1*(m + yc),0]
    
    n6=[-1,0,0]
    p6=[-1*(m + xc),0,0]
    
    
    d1=F_plane_n(x,y,z,n1,p1)
    d1x=dF_plane_nx(x,y,z,n1,p1)

    d2=F_plane_n(x,y,z,n2,p2)
    d2x=dF_plane_nx(x,y,z,n2,p2)


    d=union(d1,d2)
    dx=union_x(d1,d2,d1x,d2x)

    
    d3=F_plane_n(x,y,z,n3,p3)    
    d3x=dF_plane_nx(x,y,z,n3,p3)
   

    d=union(d,d3)    
    dx=union_x(d,d3,dx,d3x)


    d4=F_plane_n(x,y,z,n4,p4)
    d4x=dF_plane_nx(x,y,z,n4,p4)
  

    d=union(d,d4)     
    dx=union_x(d,d4,dx,d4x)


    d5=F_plane_n(x,y,z,n5,p5)  
    d5x=dF_plane_nx(x,y,z,n5,p5)
    
    
    d=union(d,d5)   
    dx=union_x(d,d5,dx,d5x)
  
     
    d6=F_plane_n(x,y,z,n6,p6)    
    d6x=dF_plane_nx(x,y,z,n6,p6)
    
    
    d=union(d,d6)  
    dx=union_x(d,d6,dx,d6x)
    
    
    return(np.nan_to_num(dx)*np.sign(d))

def DF_box_y(x,y,z,xc,yc,zc,m): 
    ''' derivative wrt y for an ADF for a box '''
    n1=[0,0,1]
    p1=[0,0,1*(m + zc)]
    
    n2=[0,1,0]
    p2=[0,1*(m + yc),0]
    
    n3=[0,0,-1]
    p3=[0,0,-1*(m + zc)]    
    
    n4=[1,0,0]
    p4=[1*(m + xc),0,0]    
    
    n5=[0,-1,0]
    p5=[0,-1*(m + yc),0]
    
    n6=[-1,0,0]
    p6=[-1*(m + xc),0,0]
        
    
    

    d1=F_plane_n(x,y,z,n1,p1)
    d1y=dF_plane_ny(x,y,z,n1,p1)
    
    
    d2=F_plane_n(x,y,z,n2,p2)
    d2y=dF_plane_ny(x,y,z,n2,p2) 
    
    d=union(d1,d2)
    dy=union_y(d1,d2,d1y,d2y)
    

    d3=F_plane_n(x,y,z,n3,p3)    
    d3y=dF_plane_ny(x,y,z,n3,p3)


    d=union(d,d3)    
    dy=union_y(d,d3,dy,d3y)
    

    d4=F_plane_n(x,y,z,n4,p4)
    d4y=dF_plane_ny(x,y,z,n4,p4)
 
    d=union(d,d4)     
    dy=union_y(d,d4,dy,d4y)


    d5=F_plane_n(x,y,z,n5,p5)  
    d5y=dF_plane_ny(x,y,z,n5,p5)

    d=union(d,d5)   
    dy=union_y(d,d5,dy,d5y)
     
    
    d6=F_plane_n(x,y,z,n6,p6)    
    d6y=dF_plane_ny(x,y,z,n6,p6)
    
    d=union(d,d6)  
    dy=union_y(d,d6,dy,d6y)

 
    return(np.nan_to_num(dy)*np.sign(d))

def DF_box_z(x,y,z,xc,yc,zc,m): 
    ''' derivative wrt z for an ADF for a box '''
    n1=[0,0,1]
    p1=[0,0,1*(m + zc)]
    
    n2=[0,1,0]
    p2=[0,1*(m + yc),0]
    
    n3=[0,0,-1]
    p3=[0,0,-1*(m + zc)]    
    
    n4=[1,0,0]
    p4=[1*(m + xc),0,0]    
    
    n5=[0,-1,0]
    p5=[0,-1*(m + yc),0]
    
    n6=[-1,0,0]
    p6=[-1*(m + xc),0,0]
            
    
    

    d1=F_plane_n(x,y,z,n1,p1)
    d1z=dF_plane_nz(x,y,z,n1,p1)
    
    

    d2=F_plane_n(x,y,z,n2,p2)
    d2z=dF_plane_nz(x,y,z,n2,p2)    
    
    d=union(d1,d2)
    dz=union_z(d1,d2,d1z,d2z)
    

    d3=F_plane_n(x,y,z,n3,p3)    
    d3z=dF_plane_nz(x,y,z,n3,p3)    

    d=union(d,d3)    
    dz=union_z(d,d3,dz,d3z)
    
    

    d4=F_plane_n(x,y,z,n4,p4)
    d4z=dF_plane_nz(x,y,z,n4,p4)    
    
    d=union(d,d4)     
    dz=union_z(d,d4,dz,d4z)



    d5=F_plane_n(x,y,z,n5,p5)  
    d5z=dF_plane_nz(x,y,z,n5,p5)
    
    d=union(d,d5)   
    dz=union_z(d,d5,dz,d5z)    
     
    

    d6=F_plane_n(x,y,z,n6,p6)    
    d6z=dF_plane_nz(x,y,z,n6,p6)
    
    d=union(d,d6)  
    dz=union_z(d,d6,dz,d6z)       
    

    return(np.nan_to_num(dz)*np.sign(d))





def F_Pryamid(x,y,z,xc,yc,zc,m,theta): 
    ''' ADF for a pyramid'''
    n1=[0,0,-1]
    p1=[0,0,zc]

    n2=[0,1*np.cos(theta),1*np.sin(theta)]
    p2=[0,1*(m + yc),(zc)]

    n3=[1*np.cos(theta),0,1*np.sin(theta)]
    p3=[(m+xc),0,(zc)]

    n4=[0,-1*np.cos(theta),1*np.sin(theta)]
    p4=[0,-1*(m + yc),(zc)]

    n5=[-1*np.cos(theta),0,1*np.sin(theta)]
    p5=[-(m+xc),0,(zc)]
    
    d1=F_plane_n(x,y,z,n1,p1)
    d2=F_plane_n(x,y,z,n2,p2)
    d3=F_plane_n(x,y,z,n3,p3)  
    d4=F_plane_n(x,y,z,n4,p4)
    d5=F_plane_n(x,y,z,n5,p5)
    
    d=union(d2,d3)
    d=union(d,d4) 
    d=union(d,d5)  
    d=union(d1,d) 
    return(d)

def DF_Pryamid_x(x,y,z,xc,yc,zc,m,theta): 
     ''' derivative wrt x for an ADF for a pyramid'''   
    
    n1=[0,0,-1]
    p1=[0,0,zc]

    n2=[0,1*np.cos(theta),1*np.sin(theta)]
    p2=[0,1*(m + yc),(zc)]

    n3=[1*np.cos(theta),0,1*np.sin(theta)]
    p3=[(m+xc),0,(zc)]

    n4=[0,-1*np.cos(theta),1*np.sin(theta)]
    p4=[0,-1*(m + yc),(zc)]

    n5=[-1*np.cos(theta),0,1*np.sin(theta)]
    p5=[-(m+xc),0,(zc)]  
    
    
    d1=F_plane_n(x,y,z,n1,p1)
    d1x=dF_plane_nx(x,y,z,n1,p1)
    
    
    d2=F_plane_n(x,y,z,n2,p2)
    d2x=dF_plane_nx(x,y,z,n2,p2) 
    
    d=union(d1,d2)
    dx=union_x(d1,d2,d1x,d2x)
    

    d3=F_plane_n(x,y,z,n3,p3)    
    d3x=dF_plane_nx(x,y,z,n3,p3)


    d=union(d,d3)    
    dx=union_x(d,d3,dx,d3x)
    

    d4=F_plane_n(x,y,z,n4,p4)
    d4x=dF_plane_nx(x,y,z,n4,p4)
 
    d=union(d,d4)     
    dx=union_x(d,d4,dx,d4x)


    d5=F_plane_n(x,y,z,n5,p5)  
    d5x=dF_plane_nx(x,y,z,n5,p5)

    d=union(d,d5)   
    dx=union_y(d,d5,dx,d5x)
     
 
    return(np.nan_to_num(dx)*np.sign(d))


def DF_Pryamid_y(x,y,z,xc,yc,zc,m,theta): 
    ''' derivative wrt y for an ADF for a pyramid'''     
    
    n1=[0,0,-1]
    p1=[0,0,zc]

    n2=[0,1*np.cos(theta),1*np.sin(theta)]
    p2=[0,1*(m + yc),(zc)]

    n3=[1*np.cos(theta),0,1*np.sin(theta)]
    p3=[(m+xc),0,(zc)]

    n4=[0,-1*np.cos(theta),1*np.sin(theta)]
    p4=[0,-1*(m + yc),(zc)]

    n5=[-1*np.cos(theta),0,1*np.sin(theta)]
    p5=[-(m+xc),0,(zc)]    
    
    
    d1=F_plane_n(x,y,z,n1,p1)
    d1y=dF_plane_ny(x,y,z,n1,p1)
    
    
    d2=F_plane_n(x,y,z,n2,p2)
    d2y=dF_plane_ny(x,y,z,n2,p2) 
    
    d=union(d1,d2)
    dy=union_y(d1,d2,d1y,d2y)
    

    d3=F_plane_n(x,y,z,n3,p3)    
    d3y=dF_plane_ny(x,y,z,n3,p3)


    d=union(d,d3)    
    dy=union_x(d,d3,dy,d3y)
    

    d4=F_plane_n(x,y,z,n4,p4)
    d4y=dF_plane_ny(x,y,z,n4,p4)
 
    d=union(d,d4)     
    dy=union_y(d,d4,dy,d4y)


    d5=F_plane_n(x,y,z,n5,p5)  
    d5y=dF_plane_ny(x,y,z,n5,p5)

    d=union(d,d5)   
    dy=union_y(d,d5,dy,d5y)
     
 
    return(np.nan_to_num(dy)*np.sign(d))


def DF_Pryamid_z(x,y,z,xc,yc,zc,m,theta): 
    ''' derivative wrt z for an ADF for a pyramid'''   
    
    n1=[0,0,-1]
    p1=[0,0,zc]

    n2=[0,1*np.cos(theta),1*np.sin(theta)]
    p2=[0,1*(m + yc),(zc)]

    n3=[1*np.cos(theta),0,1*np.sin(theta)]
    p3=[(m+xc),0,(zc)]

    n4=[0,-1*np.cos(theta),1*np.sin(theta)]
    p4=[0,-1*(m + yc),(zc)]

    n5=[-1*np.cos(theta),0,1*np.sin(theta)]
    p5=[-(m+xc),0,(zc)]   
    
    
    d1=F_plane_n(x,y,z,n1,p1)
    d1z=dF_plane_nz(x,y,z,n1,p1)
    
    
    d2=F_plane_n(x,y,z,n2,p2)
    d2z=dF_plane_nz(x,y,z,n2,p2) 
    
    d=union(d1,d2)
    dz=union_z(d1,d2,d1z,d2z)
    

    d3=F_plane_n(x,y,z,n3,p3)    
    d3z=dF_plane_nz(x,y,z,n3,p3)


    d=union(d,d3)    
    dz=union_z(d,d3,dz,d3z)
    

    d4=F_plane_n(x,y,z,n4,p4)
    d4z=dF_plane_nz(x,y,z,n4,p4)
 
    d=union(d,d4)     
    dz=union_z(d,d4,dz,d4z)


    d5=F_plane_n(x,y,z,n5,p5)  
    d5z=dF_plane_nz(x,y,z,n5,p5)

    d=union(d,d5)   
    dz=union_z(d,d5,dz,d5z)
     
 
    return(np.nan_to_num(dz)*np.sign(d))
