import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
import scipy.interpolate as spi

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul
  
def ctrl(r, L, Kv, Kh, xd, yd, x, y, d, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2) - d
    thd = m.atan2(yd-y, xd-x)
    omega = Kh*(thd-theta)
    ur, ul = v2u(v, omega, r, L)
    return ur, ul

def modCtrl(r, L, Kv, Kh, xd, yd, x, y, d, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2) - d
    v_1 = np.array([m.cos(theta), m.sin(theta)])
    v_2 = np.array([xd-x, yd-y])
    phi = m.acos(np.dot(v_1, v_2) / (np.linalg.norm(v_1) * np.linalg.norm(v_2)))
    omega = Kh*np.sign(np.cross(v_1, v_2))*phi
    ur, ul = v2u(v, omega, r, L)
    return ur, ul

ttime = 10
xarr = np.array([0, 0.5, 2, 3])
yarr = np.array([0, -1, -1, 3])
tarr = ttime*xarr/xarr[-1]

xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

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
err, robot =  vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

r = 0.5*0.195
L = 0.311
t = time.time()
tf = 15

err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

Kh = 0.8
Kv = 0.1

while (time.time()-t) < tf:
    xd = spi.splev((time.time()- t), xc)
    yd = spi.splev((time.time()- t), yc)
    ur, ul = modCtrl(r, L, Kv, Kh, xd, yd, pos[0], pos[1], 0, angle[2])
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer)
    time.sleep(0.1)

while (time.time()-t) < tf+1:
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)