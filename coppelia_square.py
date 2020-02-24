import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements

r = 0.195/2
L = 0.331
Kv = 0.8
Kh = 0.8
xd = -1.5
yd = -1.5

def v2u(v, omega, r, L):
    ur = 0.5*v + L*omega/(2*r)
    ul = 0.5*v - L*omega/(2*r)
    return ur, ul

def ctrl(r, L, Kv, Kh, xd, yd, x, y, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2)
    thd = m.atan2(yd-y, xd-x)
    omega = Kh*(thd-theta)
    ur, ul  = v2u(v, omega, r, L)
    return ur, ul

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
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

t = time.time()
# ur, ul = v2u(1.0, 0, r, L)

err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
err, ang = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

while (time.time()-t) < 30:
    err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    err, ang = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer)

    ur, ul = ctrl(r, L, Kv, Kh, xd, yd, pos[0], pos[1], ang[2])
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)

    print("Err x {}, Err y {}".format((xd-pos[1]), (yd-pos[2])))
    time.sleep(0.1)

# while (time.time()-t) < 20:
#     err = vrep.simxSetJointTargetVelocity(clientID, motorL, 0.0, vrep.simx_opmode_streaming)
#     err = vrep.simxSetJointTargetVelocity(clientID, motorR, 0.0, vrep.simx_opmode_streaming)
#     time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
