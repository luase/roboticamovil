#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 27 20:46:45 2019

@author: jpiramirez
"""

import numpy as np
import time
import math as m
import sim as vrep # access all the VREP elements
import sys

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Function for movement
def modCtrl(r, L, Kv, Kh, xd, yd, x, y, d, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2) - d
    if v > 1.0:
        v = 1.0
    v_1 = np.array([m.cos(theta), m.sin(theta)])
    v_2 = np.array([xd-x, yd-y])
    phi = m.acos(np.dot(v_1, v_2) / (np.linalg.norm(v_1) * np.linalg.norm(v_2)))
    omega = Kh*np.sign(np.cross(v_1, v_2))*phi
    if omega > 2.5:
        omega = 2.5
    elif omega < -2.5:
        omega = -2.5
    
    return v, omega

# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

Kv = 0.3
Kh = 1.0
xd = 3
yd = 3
r = 0.5*0.195
L = 0.311
errp = 10

while errp > 0.1:
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)

    uread = []
    ustate = []
    for i in range(16):
       err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
       uread.append(np.linalg.norm(point))
       ustate.append(state)
    
    v, omega = modCtrl(r, L, Kv, Kh, xd, yd, carpos[0], carpos[1], 0, carrot[2])

    if ustate[2] == True and uread[2] < 0.8:
        print('Imminent collision at '+str(uread[4]))
        omega = -1.5
        v = 0.1
    if ustate[5] == True and uread[5] < 0.8:
        print('Imminent collision at '+str(uread[4]))
        omega = 1.5
        v = 0.1

    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    time.sleep(0.1)


vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
