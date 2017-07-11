# NumPy is the fundamental package for scientific computing with Python.
# It contains among other things:
# a powerful N-dimensional array object
from gi.overrides.Gtk import Label

import numpy as np
import matplotlib.pyplot as plt
import oct2py as op
import itertools
from mpl_toolkits.mplot3d import axes3d

def DrawHexagon(P):
    T=np.vstack([np.hstack([P[0:P.shape[0]-1]]),np.hstack([P[0,:]])])
    ax.plot(T[:,0],T[:,1],T[:,2],'-b')   #'-b' same color
    return ;

def LinkDist(B,P) :
    for i in range(0,7):
        if(i==0) :
            L=(B[i,:],P[6,:])
            ax.plot(B[i,:],P[5,:]);
            print (L)
        else:
            if(i==7) :
                L = (B[i, :], P[i, :])
                #ax.plot(B[i, :], P[i, :],'-b');
                print (L)
            else:
                L = (B[i, :], P[i-1, :])
                #ax.plot(B[i, :], P[i-1, :],'-b');
                print (L)
    return ;

'''def RotateZ(LPoints,T):
    Temp=np.array(LPoints,)
    print (Temp)
    return ;'''

Theta = np.arange(0 , 360 , 120)  #arrange into [0 120 240]
#print(Theta)
DTheta = 16.22
BRad = 141.78          #Base Radiues
PRad = 101.27          #Plateform Radiues
ConnRod = 218.2587
LinkRod = 20
Height = 200
Pos = 3                   #x y z
PX = np.arange(-10 , 11)#mm
PY = np.arange(-10 , 11)    #mm
PZ=np.arange(-5,6,0.5)   #mm
RX=np.arange(-5,6,0.5)  #from mpl_toolkits.mplot3d import axes3ddegree
RY=np.arange(-5,6,0.5)  #degree
RZ=np.arange(-5,6,0.5)
BPC=np.zeros( (7,3) )#Base Plateform Coordinate
MPC=np.zeros( (7,3) )      #Moving Plateform Coordinate
LPC=np.zeros( (6,3) )      #Link Point Coordinate
BC=np.radians(np.sort(np.array([Theta ,Theta+DTheta]).ravel())) #BASE CIRCULE
#print (BC)
#PLATE CIRCULE
PC=np.radians(np.sort(np.array([Theta+60 ,Theta+DTheta+60]).ravel()))
#print (PC)
    #Calculate the Base Platform Point and Base Point Position
BPC[0:6,:]=np.transpose(np.array([BRad * np.cos(BC),BRad * np.sin(BC),np.zeros((6))]))
#print (BPC);
#Height of Moving Platform
MPC[0:6,:]=np.transpose(np.array([PRad * np.cos(PC),PRad * np.sin(PC),Height * np.ones(6)]))
MPC[6,2]=Height
#print (MPC)
#Display  the points in 3D
Point=(np.vstack([np.hstack([BPC]),np.hstack([MPC])]))#Base Points and Plateform Points
#print (Point)

fig=plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-500,500)
ax.set_ylim(-500,500)
ax.set_zlim(-500,500)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.scatter(Point[:,0],Point[:,1],Point[:,2])
#plt.plot(Point[:,0],Point[:,1],Point[:,2],'-o')
DrawHexagon(BPC)
DrawHexagon(MPC)

VirLink = LinkDist(BPC[0:7,:], MPC[0:7,:])
PlaneTheta=Theta+DTheta/2
#print (PlaneTheta)

PlaneNormal=np.transpose(np.array([BRad*np.cos(np.radians(PlaneTheta)),BRad*np.sin(np.radians(PlaneTheta)),np.zeros(3)]))
#print (PlaneNormal)

MotorPlane=np.zeros((3,5))
#print (MotorPlane)
'''for i=1:3
    MotorPlane(i,:)=createPlane(BPC(i*2-1,:),PlaneNormal(i,:));
   drawPlane3d(MotorPlane(i,:));
 end'''
for i in range(0,3):
    print (BPC[i*2,:])
    print(PlaneNormal[i,:])

Phi=np.arange(0,90,0.25)
#print (Phi)
LC=np.radians(Phi)
#LPL=[ LinkRod.*cos(LC)' zeros(1,numel(Phi))' LinkRod.*sin(LC)' ];
LPL=np.transpose(np.array([LinkRod*np.cos(LC),np.zeros((1,361)),LinkRod*np.sin(LC)]))
#print (LPL)
#LPR=[ LinkRod.*cos(pi()-LC)' zeros(1,numel(pi()-Phi))' LinkRod.*sin(pi()-LC)' ]
LPR=np.transpose(np.array([LinkRod*np.cos(np.pi-LC),np.zeros((1,361)),LinkRod*np.sin(np.pi-LC)]))
#print (LPR)
#(T1,T3,T5)=RotateZ(LPL,np.radians(PlaneTheta+90))
#(T0,T2,T4)=RotateZ(LPR,np.radians(PlaneTheta+90))
#ax.scatter(T0[:, 0],T0[:, 1], T0[:, 2])
#ax.scatter(T1[:, 0],T1[:, 1], T1[:, 2])
#ax.scatter(T2[:, 0],T2[:, 1], T2[:, 2])
plt.show()



