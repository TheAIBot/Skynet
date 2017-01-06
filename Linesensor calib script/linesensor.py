# -*- coding: utf-8 -*-
"""
Created on Thu Jan  5 10:32:16 2017

@author: nikla
"""

import numpy as np
import matplotlib.pyplot as plt

X = np.loadtxt('LogLineCalibration.txt')

s0 = X[:,0]
s1 = X[:,1]
s2 = X[:,2]
s3 = X[:,3]
s4 = X[:,4]
s5 = X[:,5]
s6 = X[:,6]
s7 = X[:,7]
speed = X[:,8]


moveIndex = 0
numSpeedValues = speed.shape[0]
for x in range(0,numSpeedValues):
    currentValue = speed[x]
    if (currentValue != 0):
        moveIndex = x #Get index for when
        break
        
stopIndex = 0
for x in range(moveIndex, speed.shape[0]):
    currentValue = speed[x]
    if (currentValue == 0):
        stopIndex = x
        break
    
    
    
s0_white_avr = np.mean( s0[0:moveIndex-1] )
s1_white_avr = np.mean( s1[0:moveIndex-1] )
s2_white_avr = np.mean( s2[0:moveIndex-1] )
s3_white_avr = np.mean( s3[0:moveIndex-1] )
s4_white_avr = np.mean( s4[0:moveIndex-1] )
s5_white_avr = np.mean( s5[0:moveIndex-1] )
s6_white_avr = np.mean( s6[0:moveIndex-1] )
s7_white_avr = np.mean( s7[0:moveIndex-1] )

s0_black_avr = np.mean( s0[stopIndex:] )
s1_black_avr = np.mean( s1[stopIndex:] )
s2_black_avr = np.mean( s2[stopIndex:] )
s3_black_avr = np.mean( s3[stopIndex:] )
s4_black_avr = np.mean( s4[stopIndex:] )
s5_black_avr = np.mean( s5[stopIndex:] )
s6_black_avr = np.mean( s6[stopIndex:] )
s7_black_avr = np.mean( s7[stopIndex:] )


def findLineParameters(white, black):
    C1 = 1
    C2 = 0
    a = (C1 - C2) / (white - black)
    
    b = -( C1*black - C2*white)/(white - black)
    
    return (a,b)

    
print("Line parameters fro the sensors: (a,b)")
s0_params = findLineParameters(s0_white_avr, s0_black_avr)
s1_params = findLineParameters(s1_white_avr, s1_black_avr)
s2_params = findLineParameters(s2_white_avr, s2_black_avr)
s3_params = findLineParameters(s3_white_avr, s3_black_avr)
s4_params = findLineParameters(s4_white_avr, s4_black_avr)
s5_params = findLineParameters(s5_white_avr, s5_black_avr)
s6_params = findLineParameters(s6_white_avr, s6_black_avr)
s7_params = findLineParameters(s7_white_avr, s7_black_avr)


f = open("linesensor_calib.txt",'w')
f.write("{0} {1}\n".format(s0_params[0],s0_params[1]))
f.write("{0} {1}\n".format(s1_params[0],s1_params[1]))
f.write("{0} {1}\n".format(s2_params[0],s2_params[1]))
f.write("{0} {1}\n".format(s3_params[0],s3_params[1]))
f.write("{0} {1}\n".format(s4_params[0],s4_params[1]))
f.write("{0} {1}\n".format(s5_params[0],s5_params[1]))
f.write("{0} {1}\n".format(s6_params[0],s6_params[1]))
f.write("{0} {1}\n".format(s7_params[0],s7_params[1]))

f.close()