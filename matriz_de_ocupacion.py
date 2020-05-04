"""
@author: luasemiaj
"""

"""
    Librerías
"""
import math as m
import matplotlib.pyplot as plt
import numpy as np
import os
from skimage.draw import line
import sys
import time
import vrep

"""
    Funciones
"""
# función world to grid
def worldToGrid(x, y, N, s):
    xr = N//2 + m.ceil(x/s)
    yr = N//2 - m.ceil(y/s) + 1
    return xr-1, yr-1 # Restamos uno porque los índices en Python inician en cero

# funcion de quaternion a 
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

"""
    Variables globales
"""
# Número de celdas de la matriz de ocupación
N = 100
# Tamaño de celda de la matriz de ocupación
s = 0.1


"""
    Inicio del programa
"""
# Conexión al simulador
vrep.simxFinish(-1) # primero terminamos toda comunicación
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # comenzamos la comunicación
if clientID!=-1:
	print ('Conectado al servidor API remoto')
else:
	print('No conectado al servidor API remoto')
	sys.exit("Sin conexión")

# Obtenemos los identificadores para los motores y el robot
err, motorIzquierdo = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorDerecho = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Obtenemos los identificadores para los sensores ultrasónicos
sensorUltrasonico = []
for i in range(1,17):
    err, sensor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    sensorUltrasonico.append(sensor)

# Inicialización de los sensores
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, sensorUltrasonico[i], vrep.simx_opmode_streaming)

# Inicialización de la posición y rotación del robot
ret, robotPosition = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)

# Cargamos el mapa existente, o en su caso, creamos uno nuevo de tamaño N por N
if os.path.exists('mapa.txt'):
    print('Cargando mapa...')
    matrizDeOcupacion = np.loadtxt('mapa.txt')
    tocc = 1.0*(matrizDeOcupacion > 0.5)
    matrizDeOcupacion[matrizDeOcupacion > 0.5] = 0
else:
    print('Creando mapa...')
    matrizDeOcupacion = 0.5*np.ones((N, N))
    tocc = np.zeros((N, N))

# Contamos el tiempo
t = time.time()
# Por 20s dejamos que el robot navegue utilizando navegación reactiva y creamos el mapa.
while time.time()-t < 30:
    # Detectamos la posición y orientación del robot
    ret, robotPosition = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    # Leemos cada sensor ultrasónico
    estadosDeteccion = []
    puntosDetectados = []
    for i in range(16):
        # Leemos el sensor ultrasónico
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, sensorUltrasonico[i], vrep.simx_opmode_buffer)
        # Agregamos el estado de detección al vector estadosDeteccion
        estadosDeteccion.append(state)
        # Agregamos el punto normalizado al vector puntosDetectados
        puntosDetectados.append(np.linalg.norm(point))
        """
            Creación del mapa
        """
        xr, yr = worldToGrid(robotPosition[0], robotPosition[1], N, s)
        print('Rows {}   Cols {}'.format(yr, xr))
        if xr >= N:
            xr = N
        if yr >= N:
            yr = N
        # Marcamos el espacio como vacio
        matrizDeOcupacion[yr-1, xr-1] = 0
        ret, srot = vrep.simxGetObjectQuaternion(clientID, sensorUltrasonico[i], -1, vrep.simx_opmode_blocking)
        ret, spos = vrep.simxGetObjectPosition(clientID, sensorUltrasonico[i], -1, vrep.simx_opmode_blocking)
        R = q2R(srot[0], srot[1], srot[2], srot[3])
        spos = np.array(spos).reshape((3,1))
        # if i % 2 != 0:
        #     continue
        if state == True:
            opos = np.array(point).reshape((3,1))
            pobs = np.matmul(R, opos) + spos
            xo, yo = worldToGrid(pobs[0], pobs[1], N, s)
            if xo >= N:
                xo = N
            if yo >= N:
                yo = N
            rows, cols = line(yr-1, xr-1, yo-1, xo-1)
            matrizDeOcupacion[rows, cols] = 0
            tocc[yo-1, xo-1] = 1
        else:
            opos = np.array([0,0,1]).reshape((3,1))
            pobs = np.matmul(R, opos) + spos
            xo, yo = worldToGrid(pobs[0], pobs[1], N, s)
            if xo >= N:
                xo = N
            if yo >= N:
                yo = N
            rows, cols = line(yr-1, xr-1, yo-1, xo-1)
            matrizDeOcupacion[rows, cols] = 0

    # Velocidad inicial para ambos motores
    uIzq = 2.0
    uDer = 2.0
    # Ganancias para el algoritmo de Braitenberg
    gananciasIzq = np.linspace(0, -1, 8)
    gananciasDer = np.linspace(-1, 0, 8)
    for i in range(8):
        if estadosDeteccion[i]:
            uIzq = uIzq + gananciasIzq[i]*(1 - puntosDetectados[i])
            uDer = uDer + gananciasDer[i]*(1 - puntosDetectados[i])

    errf = vrep.simxSetJointTargetVelocity(clientID, motorIzquierdo, uIzq, vrep.simx_opmode_oneshot)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorDerecho, uDer, vrep.simx_opmode_oneshot)

# Mostrar los resultados usando plt
plt.figure(1)
plt.imshow(tocc+matrizDeOcupacion)
plt.show()

# Guardar el mapa
np.savetxt('mapa.txt', tocc+matrizDeOcupacion)

# Terminar la simulación
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
