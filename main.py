#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from gaitPlanner import trotGait

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


cubeStartPos = [0,0,0]
FixedBase = False #if fixed no plane is imported
if (FixedBase == False):
    p.loadURDF("plane.urdf")
boxId = p.loadURDF("C:/Users/kagan/AppData/Local/Programs/Microsoft VS Code/PROJECTS/(Unsaved)/(Unsaved).urdf",cubeStartPos, useFixedBase=1)

jointIds = []
paramIds = [] 
time.sleep(0.5)
for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 2


pybulletDebug = pybulletDebug()
robotKinematics = robotKinematics()
trot = trotGait() 

#robot properties
"""initial foot position"""
#foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
Xdist = 0.20
Ydist = 0.15
height = 0.15
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                         [ Xdist/2 ,  Ydist/2 , -height],
                         [-Xdist/2 , -Ydist/2 , -height],
                         [-Xdist/2 ,  Ydist/2 , -height]])

T = 0.5 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)

p.setRealTimeSimulation(1)
    
while(True):
    lastTime = time.time()
    pos , orn , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)  
    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    bodytoFeet = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

#####################################################################################
#####   kinematics Model: Input body orientation, deviation and foot position    ####
#####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)
        
    #move movable joints
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    
    
#    print(time.time() - lastTime)
p.disconnect()
