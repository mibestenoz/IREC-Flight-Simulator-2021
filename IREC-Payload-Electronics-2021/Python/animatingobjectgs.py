from vpython import *
import numpy as np
x = box(length=6,width=2,height=.2,opacity=.2)
y = cylinder(length=6,radius=.5,pos=vector(-3,0,0),color=color.red,opacity=.5)
z = sphere(radius=.75,pos=vector(-3,0,0),color=color.red,opacity=.5)
xArrow = arrow(length=2,axis=vector(1,0,0),color=color.red)
yArrow = arrow(length=2,axis=vector(0,1,0),color=color.green)
zArrow = arrow(length=2,axis=vector(0,0,1),color=color.blue)
while True:
    for j in np.arange(1,6,.05):
        rate(20)
        y.length=j
    for j in np.arange(6,1,-.05):
        rate(20)
        y.length=j
    
