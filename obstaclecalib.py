import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
from q2R import q2R

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot  = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

err, psr = vrep.simxGetObjectPosition(clientID, usensor[2], robot, vrep.simx_opmode_streaming)
err, psr = vrep.simxGetObjectPosition(clientID, usensor[2], robot, vrep.simx_opmode_buffer)

err, osr = vrep.simxGetObjectOrientation(clientID, usensor[2], robot, vrep.simx_opmode_streaming)
err, osr = vrep.simxGetObjectOrientation(clientID, usensor[2], robot, vrep.simx_opmode_buffer)

Rsr = q2R(osr[0], osr[1], osr[2], osr[3])
tsr = np.array(psr).resize((3,1))

t = time.time()

while (time.time()-t) < 5:
    smeasure = []
    sstate = []
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
        sstate.append(state)
    print(smeasure[2], sstate[2])
    point_s = np.matrix([[0, 0, smeasure[2]]]).T
    
    time.sleep(0.1)

while (time.time()-t) < 5:
    time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
