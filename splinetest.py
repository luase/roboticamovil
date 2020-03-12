import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

ttime = 10
xarr = np.array([0, 0.5, 2, 3])
yarr = np.array([0, -1, -1, 3])
tarr = ttime*xarr/xarr[-1]

tnew = np.linspace(0, 10, 100)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

xnew = spi.splev(tnew, xc)
ynew = spi.splev(tnew, yc)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, '.')
plt.show()