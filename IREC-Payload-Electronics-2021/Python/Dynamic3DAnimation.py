from vpython import *
leftwall = box(pos=vector(-5,0,0),length=.1,width=10,height=10,color=color.white,opacity=.4)
rightwall = box(pos=vector(5,0,0),length=.1,width=10,height=10,color=color.white,opacity=.4)
topwall = box(pos=vector(0,5,0),length=10,width=10,height=.1,color=color.white,opacity=.4)
bottomwall = box(pos=vector(0,-5,0),length=10,width=10,height=.1,color=color.white,opacity=.4)
frontwall = box(pos=vector(0,0,5),length=10,width=.1,height=10,color=color.white,opacity=.4)
backwall = box(pos=vector(0,0,-5),length=10,width=.1,height=10,color=color.white,opacity=.4)
ball = sphere(radius=.5,color=color.red)

xpos=0
ypos=1
zpos=-2
xchange=.1
ychange=.1
zchange=.1

while True:
    rate(60)
    ball.pos=vector(xpos,ypos,zpos)
    xpos = xpos + xchange
    ypos = ypos + ychange
    zpos = zpos + zchange
    if xpos>=4.55:
        xchange=-.1
    if xpos<=-4.555:
        xchange=.1
    if ypos>=4.55:
        ychange=-.1
    if ypos<=-4.55:
        ychange=.1
    if zpos>=4.55:
        zchange=-.1
    if zpos<=-4.55:
        zchange=.1
