#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.

from hpp.corbaserver.robot_2d import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('robot_2d')
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('robot_2d_description','cylinder_obstacle','')
robot.setJointBounds ('j_translation_x', [-9, 6])
robot.setJointBounds ('j_translation_y', [-2, 4])

# q = [x, y] # limits in URDF file
q1 = [-7, 3]; q2 = [-8, 2]; q3 = [-6, 2]; q4 = [-7, 1]; q5 = [-5, -1]; q6 = [-8, -1]
q7 = [-4, 1]; q8 = [-2, -1]; q9 = [0.2, 1.6]; q10 = [1, -1]; q11 = [4, 1]; q12 = [5, -1];
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q4); ps.addGoalConfig (q5); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q5); ps.addGoalConfig (q6); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q6); ps.addGoalConfig (q7); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q7); ps.addGoalConfig (q8); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q8); ps.addGoalConfig (q9); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q9); ps.addGoalConfig (q10); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q10); ps.addGoalConfig (q11); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q11); ps.addGoalConfig (q12); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q12); ps.solve (); # 11
cl.problem.pathLength(11)
len(ps.getWaypoints (11))

ps.addPathOptimizer("Prune")
ps.optimizePath (11)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (11)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (11)
ps.pathLength(ps.numberPaths()-1)
timeValuesRS = cl.problem.getTimeValues ()
gainValuesRS = cl.problem.getGainValues ()
newGainValuesRS = ((1-np.array(gainValuesRS))*100).tolist()

ps.optimizePath (11)
ps.pathLength(ps.numberPaths()-1)
timeValuesRS2 = cl.problem.getTimeValues ()
gainValuesRS2 = cl.problem.getGainValues ()
newGainValuesRS2 = ((1-np.array(gainValuesRS2))*100).tolist()

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (11)
ps.pathLength(ps.numberPaths()-1)



## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 10 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 11, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 11, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-.004, tGB+0.004, 45, 102])
plt.xlabel('t (s)'); plt.ylabel('Relative length reduction (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', 1.5)
plt = curvPlot (plt, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt = curvSdPlot (plt, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.002)
plt = curvPlot (plt, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt = curvSdPlot (plt, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.002)
plt = curvPlot (plt, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
plt.axis([-.004, max(tGB,max(tGB2,tGB3))+0.004, 45, 102])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tGB, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tGB, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tGB, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------

cl.problem.setAlphaInit (0.05)
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4)
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------

import matplotlib.pyplot as plt
from mutable_trajectory_plot import planarPlot, addNodePlot
from parseLog import parseCollConstrPoints
num_log = 30463
contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
plt = planarPlot (cl, 11, ps.numberPaths()-1, plt, 1.5, 7)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt.show()


len(cl.problem.nodes ())
len(ps.getWaypoints (0))
cl.problem.getIterationNumber()

"""
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle2","cylinder_obstacle2")
"""

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 20855
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:223: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:217: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:179: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:181: finish path parsing'
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, 0)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, 0)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, 0)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, 0)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, 0)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, 0)

plt = planarPlot (cl, 11, 12, plt, 8) # initialize 2D plot with obstacles and path
plt = addPathPlot (cl, x0Path, '0.9', 1, plt)
plt = addPathPlot (cl, x1Path, '0.8', 1, plt)
plt = addPathPlot (cl, x2Path, '0.7', 1, plt)
plt = addPathPlot (cl, x3Path, '0.6', 1, plt)
plt = addPathPlot (cl, x4Path, '0.5', 1, plt)
plt = addPathPlot (cl, x5Path, '0.4', 1, plt)
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt.show() # will reset plt
"""
plt = planarPlot (cl, 11, 12, plt, 8) # initialize 2D plot with obstacles and path
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt = addPathPlot (cl, x0Path, 'm', 1, plt)
plt = addPathPlot (cl, x1Path, 'g', 1, plt)
plt = addPathPlot (cl, x2Path, 'b', 1, plt)
plt = addPathPlot (cl, x3Path, 'y', 1, plt)
plt = addPathPlot (cl, x4Path, 'c', 1, plt)
plt = addPathPlot (cl, x5Path, '0.75', 1, plt)
plt.show() # will reset plt
"""



#####################################################################

## DEBUG commands
cl.robot.setCurrentConfig(q2)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()





## Debug Optimization Tools ##############
import matplotlib.pyplot as plt
num_log = 20855
from parseLog import parseCollConstrPoints, parseNodes
from mutable_trajectory_plot import planarPlot, addNodePlot, addCircleNodePlot, addNodeAndLinePlot, addCorbaPathPlot

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '94: x1 in J1 = (')
x2_J1 = parseCollConstrPoints (num_log, '95: x2 in J1 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

plt = planarPlot (cl, 11, ps.numberPaths()-1, plt, 1.5, 7)
i=12; iMax = ps.numberPaths()-1;
while (i < iMax):
    plt = addCorbaPathPlot (cl, i, '0.75', 0.9, plt)
    i = i + 2

#plt = addCorbaPathPlot (cl, 0, 'rk', 1.6, plt)
plt = addCorbaPathPlot (cl, ps.numberPaths()-1, 'k', 1.5, plt)
plt = addCircleNodePlot (collConstrNodes, 'b', 0.14, plt)
#plt = addCircleNodePlot (collNodes, 'r', 0.14, plt)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt = addNodeAndLinePlot (x1_J1, x2_J1, 'ro', 'bo', 4, 'r', 1.3, plt)
plt = addNodeAndLinePlot (x1_J2, x2_J2, 'ro', 'bo', 4, 'r', 1.3, plt)

plt = addNodePlot (collConstrNodes, 'ko', '', 5, plt)

plt.show()


## same with viewer !
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")

from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)


sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

dt = 0.2
plot2DBaseCurvPath (r, cl, dt, 11, "curvPathInitial", [1,0.3,0,1])
plot2DBaseCurvPath (r, cl, dt, 12, "curvPathRS", [0,0,1,1])
plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPathGB", [0,1,0,1])

