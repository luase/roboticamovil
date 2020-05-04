import numpy as np
import time
import math as m
import sys
import vrep # access all the VREP elements

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1/2*(x**2+y**2)
    return R

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
err = 1
while err:
    err, psr = vrep.simxGetObjectPosition(clientID, usensor[2], robot, vrep.simx_opmode_buffer)
err, qsr = vrep.simxGetObjectQuaternion(clientID, usensor[2], robot, vrep.simx_opmode_streaming)
err = 1
while err:
    err, qsr = vrep.simxGetObjectQuaternion(clientID, usensor[2], robot, vrep.simx_opmode_buffer)

Rsr = q2R(qsr[0], qsr[1], qsr[2], qsr[3])
tsr = np.array(psr).reshape((3,1))

t = time.time()

while (time.time()-t) < 10:
    smeasure = []
    sstate = []
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        smeasure.append(np.linalg.norm(point))
        sstate.append(state)
    print("{} {}".format(smeasure[2], sstate[2]))
    point_s = np.matrix([[0,0,smeasure[2]]]).T
    if sstate[2]:
        point_r = np.matmul(Rsr,point_s) + tsr
        print(point_r)
    time.sleep(0.1)

while (time.time()-t) < 20:
    time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
