import vpython
import time
from vpython import *
scene.range = 5
bBoard = box(pos=vector(0,.1,-2),length=6,width=2.75,height=.2,color=color.white)
Xarrow = arrow(axis=vector(1,0,0),length=5,shaftwidth=.1,color=color.red)
Yarrow = arrow(axis=vector(0,-1,0),length=5,shaftwidth=.1,color=color.green)
Zarrow = arrow(axis=vector(0,0,-1),length=5,shaftwidth=.1,color=color.blue)
arduino = box(pos=vector(0,.1,2),length=5,width=2.25,height=.2,color=color.blue)
yellowplate = box(length=7,width=7,height=.1,color=color.yellow,pos=vector(0,0,0))
bno = box(color=color.blue,length=1,width=1,height=.1,pos=vector(-2,.2,-2))
