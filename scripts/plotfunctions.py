#/usr/bin/env python
# Script: functions to plot graphs with Matplotlib for path-optimization comparison (GB-RS-PRS)
# User has to call himself "plt.show()"

from __future__ import division
#import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math


# --------------------------------------------------------------------#

def curvPlot (plt, tGB, timeValues, newGainValues, symbol, color, lw):
    vectorLength = len(timeValues)
    ti = timeValues[vectorLength-1]
    gvi = newGainValues[vectorLength-1]
    plt.plot(ti, gvi, color+symbol)
    plt.plot([ti,tGB], [gvi,gvi], color, linewidth=lw)
    for i in np.arange(0, vectorLength-1):
        ti = timeValues[i]; tii = timeValues[i+1]; gvi = newGainValues[i]
        plt.plot([ti,tii], [gvi,gvi], color, linewidth=lw)
        plt.plot(ti, gvi, color+symbol)
    
    return plt

# --------------------------------------------------------------------#
# just sd vertical lines
def curvSdPlot (plt, tGB, tReduceVector, meanReduceVector, sdReduceVector, color, lw, lsd):
    vectorLength = len(tReduceVector)
    iFinal = vectorLength-1
    middleti = (tReduceVector[iFinal] + tGB)*0.5
    gvi = meanReduceVector[iFinal]; sdi = sdReduceVector[iFinal]
    plt.plot([middleti,middleti], [gvi-sdi,gvi+sdi], color, linewidth=lw)
    plt.plot([middleti-lsd,middleti+lsd], [gvi+sdi,gvi+sdi], color, linewidth=lw)
    plt.plot([middleti-lsd,middleti+lsd], [gvi-sdi,gvi-sdi], color, linewidth=lw)
    for i in np.arange(0, vectorLength-1):
        ti = tReduceVector[i]; tii = tReduceVector[i+1]
        gvi = meanReduceVector[i]; sdi = sdReduceVector[i]
        middleti = (ti + tii)*0.5
        plt.plot([middleti,middleti], [gvi-sdi,gvi+sdi], color, linewidth=lw) # vertical
        plt.plot([middleti-lsd,middleti+lsd], [gvi+sdi,gvi+sdi], color, linewidth=lw)
        plt.plot([middleti-lsd,middleti+lsd], [gvi-sdi,gvi-sdi], color, linewidth=lw)
    
    return plt

# --------------------------------------------------------------------#
# warning: between [0;timeValues[i]], this will return -1...
def getValIndex (timeValues, t):
    i = 0
    tcomp = timeValues[i]
    if (t > timeValues[len(timeValues)-1]):
        return len(timeValues)-1
    while (t > tcomp):
        i = i + 1
        tcomp = timeValues[i]
    #print "t= " + str(t) + ", tcomp= " + str(tcomp) + ", index= " + str(i-1)
    return i-1

# --------------------------------------------------------------------#

# optimize nbOptimzations times AND Concatenate values (return list of list):
def optAndConcatenate (cl, ps, idPathToOpt, nbOptimzations, optimizerType, globalTimeValues, globalGainValues):
    ps.clearPathOptimizers(); ps.addPathOptimizer(optimizerType)
    for i in np.arange(0, nbOptimzations):
        ps.optimizePath (idPathToOpt)
        timeValues = cl.problem.getTimeValues ()
        gainValues = cl.problem.getGainValues ()
        newGainValues = ((1-np.array(gainValues))*100).tolist()
        globalTimeValues.append(timeValues)
        globalGainValues.append(newGainValues)
        #print globalTimeValues

# --------------------------------------------------------------------#
# On each sample of tVec, compute means (and sd) of all RS values
# (corresponding to the time-sample)
# we could compute less values (and more easily get the middles)
def computeMeansVector (nbOpt, tVec, moyVector, sdVector, globalTimeValues, globalGainValues):
    for t in tVec:
        moy = 0
        sumSquareVals = 0
        for j in np.arange(0,nbOpt):
            indexInVector = getValIndex(globalTimeValues[j],t)
            if (indexInVector == -1):
                moy = moy + 1/nbOpt*100
                sumSquareVals = sumSquareVals + 100**2
            else:
                moy = moy + 1/nbOpt*globalGainValues[j][indexInVector]
                sumSquareVals = sumSquareVals + globalGainValues[j][indexInVector]**2
        moyVector.append(moy)
        var = 1/nbOpt*sumSquareVals - moy**2
        if (var < 0):
            var = 0
        sdVector.append(math.sqrt(var))
		
# --------------------------------------------------------------------#
# compute middles (their time, mean and their SD) 
# to plot SD directly (not several SD per same mean gain value)
def reducedVectors (timeValues, meanValues, sdValues, tReduceVector, meanReduceVector, sdReduceVector):
    vectorLength = len(timeValues)
    tReduceVector.append(0)
    meanReduceVector.append(meanValues[0]); sdReduceVector.append(sdValues[0]) # so vectors have same length
    for i in np.arange(0, vectorLength-1):
        ti = timeValues[i]; tii = timeValues[i+1]
        gvi = meanValues[i]; gvii = meanValues[i+1];
        sdi = sdValues[i]; sdii = sdValues[i+1]
        if (math.fabs(gvi - gvii) > 1e-2):
            #tReduceVector.append(ti); meanReduceVector.append(gvi); sdReduceVector.append(sdi);
            tReduceVector.append(tii); meanReduceVector.append(gvii); sdReduceVector.append(sdii);
    
    tReduceVector.append(timeValues[vectorLength-1])
    meanReduceVector.append(meanValues[vectorLength-1]); sdReduceVector.append(sdValues[vectorLength-1]) # so vectors have same length


