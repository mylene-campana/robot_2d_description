#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.

from hpp.corbaserver.robot_2d import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
import matplotlib.pyplot as plt
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('robot_2d')
robot.setJointBounds('j_translation_x', [-100, 100])
robot.setJointBounds('j_translation_y', [-2, 2])
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('robot_2d_description','box','')

"""
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","box","box")
"""

# q = [x, y] # limits in URDF file
dist=100
q1 = [-dist, 1]; q2 = [-0.8, 1]; q3 = [0.5, 1.5]; q4 = [0.7, 0]; q5 = [-0.7, -1.7];
q6 = [0.4,-1.5]; q7 = [-0.6, -0.2]; q8 = [0.2, 0.2]; q9 = [1, -1]; q10 = [dist, -1]; 

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q4); ps.addGoalConfig (q5)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q5); ps.addGoalConfig (q6)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q6); ps.addGoalConfig (q7)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q7); ps.addGoalConfig (q8)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q8); ps.addGoalConfig (q9)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q9); ps.addGoalConfig (q10)
ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q10); ps.solve ();
print ps.pathLength(9)

ps.addPathOptimizer("GradientBased")
ps.optimizePath (9)
ps.numberPaths()
print ps.pathLength(ps.numberPaths()-1)

"""
# pp(9) = p0 final
ps.optimizePath(9) # pp(10) = p1 final

ps.pathLength(9)
ps.pathLength(10)


ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (9)
ps.pathLength(10)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (9)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","box","box")


import numpy as np
dt = 0.06
nPath = ps.numberPaths()-1
lineNamePrefix = "pathyyjf"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0.2],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0.2],[0,0.3,1,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

nPath = 9
lineNamePrefix = "jhk"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0.12],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0.12],[1,0.4,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 16020
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarBoxPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:228: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:222: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:184: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:186: finish path parsing'
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, 0)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, 0)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, 0)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, 0)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, 0)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, 0)

plt = planarBoxPlot (cl, 9, 10, plt, 2.5) # initialize 2D plot with obstacles and path
plt = addPathPlot (cl, x0Path, '0.9', 1, plt)
plt = addPathPlot (cl, x1Path, '0.8', 1, plt)
plt = addPathPlot (cl, x2Path, '0.7', 1, plt)
plt = addPathPlot (cl, x3Path, '0.6', 1, plt)
plt = addPathPlot (cl, x4Path, '0.5', 1, plt)
plt = addPathPlot (cl, x5Path, '0.4', 1, plt)
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt.show() # will reset plt

ps.optimizePath(10)
plt = planarBoxPlot (cl, 10, 11, plt, 2.5)
plt.show()

#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( ps.configParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
"""
