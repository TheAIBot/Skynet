from pylab import *
import numpy as np
import scipy as scipy
from scipy.optimize import curve_fit

data = np.loadtxt("2.2-log.txt")

count = 0
for i in range(0, data.shape[0] - 1):
    if data[i, 0] == 0:
        count = count + 1

mat = np.empty((count,5))
distmat = np.empty((count, 1))
index = 0
newDist = 0
distIndex = -1
print(data.shape)
for i in range(0, data.shape[0] - 1):
    if data[i, 0] == 0:
        if newDist == 1:
            distIndex = distIndex + 1
            newDist = 0
        mat[index, 0] = data[i, 1]
        mat[index, 1] = data[i, 2]
        mat[index, 2] = data[i, 3]
        mat[index, 3] = data[i, 4]
        mat[index, 4] = data[i, 5]
        distmat[index] = 75 - (10 * distIndex)
        index = index + 1
        #if index == 700:
        #    break
    else:
        newDist = 1
print(index)
#print(mat)
#print(distmat)

def func(x, ka, kb):
    return (ka/x) + kb

val = curve_fit(func, distmat[:, 0], mat[:, 2])
ka = val[0][0]
kb = val[0][1]
print("ka = " + str(ka))
print("kb = " + str(kb))


figure()
plot(distmat[:, 0], mat[:, 2], "x", label = "log points")
plot(range(15, 75 + 1), func(range(15, 75 + 1), ka, kb), label = "fitted curve")
xlabel("distance, m")
ylabel("ir output")
title("ir fitted curve")
#savefig("2-2-fitted-curve.pdf")

#Print ka and kb to file
f = open("irSensorCalib.txt",'w')
f.write("{0} {1}\n".format(ka,kb))
f.close()
