import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements
import scipy.interpolate as spi

# Velocity and angle function
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

# File reading and array creations
xlist = []
ylist = []
with open('input.txt') as input_file:
    for line in input_file:
        numbers = line.split(' ')
        xlist.append( float(numbers[0].strip()) )
        ylist.append( float(numbers[1].strip()) )
print('Vector x de puntos', xlist)
print('Vector y de puntos', ylist)

t = time.time()
tf = ((xlist[0]-xlist[-1])**2+(ylist[0]-ylist[-1])**2)**(1/2)*5
print('Tiempo que se tiene:', tf)

ttime = tf
xarr = np.array(xlist)
yarr = np.array(ylist)
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

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

Kv = 0.1
Kh = 0.8
r = 0.5*0.195
L = 0.311
errp = 10

while (time.time()-t) < tf:

    err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer)
    xd = spi.splev((time.time()- t), xc)
    yd = spi.splev((time.time()- t), yc)

    errp = m.sqrt((xd-pos[0])**2 + (yd-pos[1])**2)
    
    uread = []
    ustate = []
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
        uread.append(np.linalg.norm(point))
        ustate.append(state)

    v, omega = modCtrl(r, L, Kv, Kh, xd, yd, pos[0], pos[1], 0, angle[2])

    if ustate[2] == True and uread[2] < 0.4:
        print('Imminent collision at '+str(uread[4]))
        omega = -1.5
        v = 0.1
    if ustate[5] == True and uread[5] < 0.4:
        print('Imminent collision at '+str(uread[4]))
        omega = 1.5
        v = 0.1

    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)

    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    time.sleep(0.1)
    if errp > 0.1:
        continue

err = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
err = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)