# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 14:10:02 2021

@author: dmulr
"""

import numpy as np
import objects as sim_obj
import matplotlib.pyplot as plt



''' wrench 
x=[0,.56,.66,1,1,.75,.75,1,1,.66,.56,0,0]
y=[.125,.125,0,0,.125,.125,.375,.375,.5,.5,.375,.375,.125]
'''

''' wrench no mouth
x=[0,.56,.66,1,1,.66,.56,0,0]
y=[.125,.125,0,0,.5,.5,.375,.375,.125]
'''


''' corner shape
segments=np.array([[-1,-1,1,-1],[1,-1,1,0],[1,0,0,0],[0,0,0,1],[0,1,-1,1],[-1,1,-1,-1]])
'''

''' rectangle
w=1.5
h=1
x=[w,-w,-w,w,w]
y=[h,h,-h,-h,h]
'''

const=2.35
x2=[0,.56,.66,1,1,.66,.56,0,0]
y2=[.125,.125,0,0,.5,.5,.375,.375,.125]
x2=np.dot(const,x2)
y2=np.dot(const,y2)
xp2=np.sum(x2[0:-1])/len(x2[0:-1])
yp2=np.sum(y2[0:-1])/len(y2[0:-1])


x2=x2-xp2
y2=y2-yp2
(segments)=sim_obj.create_segment(x2,y2)
#sim_obj.plot_line(x,y)

vertices=np.vstack([[x2],[y2]])
print(sim_obj.shoelace(vertices))



name='wrench'
np.savez('shapes/'+name+'.npz',segments=segments,xp=x2,yp=y2)



x=[0,.56,.66,1,1,.75,.75,1,1,.66,.56,0,0]
y=[.125,.125,0,0,.125,.125,.375,.375,.5,.5,.375,.375,.125]
x=np.dot(const,x)
y=np.dot(const,y)
xp=np.sum(x[0:-1])/len(x[0:-1])
yp=np.sum(y[0:-1])/len(y[0:-1])

x=x-xp2
y=y-yp2
plt.plot(x,y,x2,y2)
#fx=6
#fy=6
#d=2.5
#sim_obj.plot_line(x,y,fx,fy,d)
(segments)=sim_obj.create_segment(x,y)
#sim_obj.plot_line(x,y)

vertices=np.vstack([[x],[y]])
print(sim_obj.shoelace(vertices))



name='wrench'
np.savez('shapes/'+name+'.npz',segments=segments,xp=x,yp=y)