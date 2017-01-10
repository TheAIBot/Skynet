from pylab import *
import numpy as np
from scipy.optimize import curve_fit

data_front = np.loadtxt("IR_front_raw.txt")
data_right = np.loadtxt("IR_right_raw.txt")
data_left = np.loadtxt("IR_left_raw.txt")

#Load front raw data
count = 0
for i in range(0, data_front.shape[0] - 1):
    if data_front[i, 0] == 0:
        count = count + 1

mat_front = np.empty((count,3))
distmat = np.empty((count, 1))
index = 0
newDist = 0
distIndex = -1
print(data_front.shape)
for i in range(0, data_front.shape[0] - 1):
    if data_front[i, 0] == 0:
        if newDist == 1:
            distIndex = distIndex + 1
            newDist = 0
        #mat[index, 0] = data_front[i, 1]
        mat_front[index, 0] = data_front[i, 2]
        mat_front[index, 1] = data_front[i, 3]
        mat_front[index, 2] = data_front[i, 4]
        #mat[index, 4] = data_front[i, 5]
        distmat[index] = 75 - (10 * distIndex)
        index = index + 1
        #if index == 700:
        #    break
    else:
        newDist = 1
print(index)
#print(mat)
#print(distmat)

#Load right raw data
count_r = 0
for i in range(0, data_right.shape[0]):
    if data_right[i,0] == 1:
        count_r = count_r + 1

newDist = 0
distIndex = -1
mat_right = np.empty((count_r,1))
distmat_r = np.empty((count_r,1))
index = 0
newDist = 0
distIndex = -1
for i in range(0, data_right.shape[0] - 1):
    if data_right[i, 0] == 1:
        if newDist == 1:
            distIndex = distIndex + 1
            newDist = 0
        mat_right[index, 0] = data_right[i, 5]
        distmat_r[index] = 75 - (10 * distIndex)
        index = index + 1
        #if index == 700:
        #    break
    else:
        newDist = 1

#Load left raw data
count_l = 0
for i in range(0, data_left.shape[0]):
    if data_left[i,0] == 1:
        count_l = count_l + 1

newDist = 0
distIndex = -1
mat_left = np.empty((count_l,1))
distmat_l = np.empty((count_l,1))
index = 0
newDist = 0
distIndex = -1
for i in range(0, data_left.shape[0] - 1):
    if data_left[i, 0] == 1:
        if newDist == 1:
            distIndex = distIndex + 1
            newDist = 0
        mat_left[index, 0] = data_left[i, 1]
        distmat_l[index] = 75 - (10 * distIndex)
        index = index + 1
        #if index == 700:
        #    break
    else:
        newDist = 1

def func(x, ka, kb):
    return (ka/x) + kb

def getSensorCalibConstants(sensorIndex):
    if sensorIndex == 0:
        val = curve_fit(func, distmat_l[:, 0], mat_left[:, 0])
        ka = val[0][0]
        kb = val[0][1]
        return (ka, kb)
        
    if sensorIndex == 4:
        val = curve_fit(func, distmat_r[:, 0], mat_right[:, 0])
        ka = val[0][0]
        kb = val[0][1]
        return (ka, kb)
        
    else:
        sensorIndex = sensorIndex - 1 #Reset to work with mat_front
        val = curve_fit(func, distmat[:, 0], mat_front[:, sensorIndex])
        ka = val[0][0]
        kb = val[0][1]
        return (ka, kb)
    
    

    
for i in range(0,5):
    print(i," : ", getSensorCalibConstants(i))


#figure()
#plot(distmat[:, 0], mat[:, 2], "x", label = "log points")
#plot(range(15, 75 + 1), func(range(15, 75 + 1), ka, kb), label = "fitted curve")
#xlabel("distance, m")
#ylabel("ir output")
#title("ir fitted curve")
#savefig("2-2-fitted-curve.pdf")

#Print ka and kb to file
f = open("irSensorCalib.txt",'w')
for i in range(0,5):
    ka, kb = getSensorCalibConstants(i)
    f.write("{0} {1}\n".format(ka,kb))
f.close()
