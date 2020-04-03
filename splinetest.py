import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

xlist = []
ylist = []
with open('input.txt') as input_file:
    for line in input_file:
        numbers = line.split(' ')
        xlist.append( float(numbers[0].strip()) )
        ylist.append( float(numbers[1].strip()) )

ttime = 1000
xarr = np.array(xlist)
yarr = np.array(ylist)
tarr = ttime*xarr/xarr[-1]
print (xarr, yarr, tarr)

tnew = np.linspace(0, 1000, 100)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

xnew = spi.splev(tnew, xc)
ynew = spi.splev(tnew, yc)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
plt.show()