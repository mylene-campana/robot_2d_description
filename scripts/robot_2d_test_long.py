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

ps.addPathOptimizer("GradientBased")
#ps.addPathOptimizer("Prune")
cl.problem.optimizePath(11)

cl.problem.pathLength(11)
cl.problem.pathLength(12)

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
num_log = 26871
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



from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")

# plot paths in viewer
import numpy as np
dt = 0.2
nPath = 11
lineNamePrefix = "initial_path"
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve (lineNamePrefix,points,[1,0.3,0,1])
r.client.gui.addToGroup (lineNamePrefix, r.sceneName)

nPath = 12
lineNamePrefix = "optim_path"
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve (lineNamePrefix,points,[0.3,1,0.3,1])
r.client.gui.addToGroup (lineNamePrefix, r.sceneName)



## Debug Optimization Tools ##############
import matplotlib.pyplot as plt
num_log = 30694
from parseLog import parseCollConstrPoints, parseNodes
from mutable_trajectory_plot import planarPlot, addNodePlot, addCircleNodePlot, addNodeAndLinePlot, addCorbaPathPlot

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '94: x1 in J1 = (')
x2_J1 = parseCollConstrPoints (num_log, '95: x2 in J1 = (')
x1_J2 = parseCollConstrPoints (num_log, '112: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '113: x2 in J2 = (') #x2_J2 <=> contactPoints

u_vectors_J1 = x2_J1 - x1_J1; u_vectors_J2 = x2_J2 - x1_J2

collConstrNodes = parseNodes (num_log, '186: qFree_ = ')
collNodes = parseNodes (num_log, '178: qColl = ')

plt = planarPlot (cl, 11, 15, plt, 7)
plt = addCorbaPathPlot (cl, 13, '0.75', plt)
plt = addCorbaPathPlot (cl, 14, '0.5', plt)
plt = addCircleNodePlot (collConstrNodes, 'b', 0.14, plt)
#plt = addCircleNodePlot (collNodes, 'r', 0.14, plt)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt = addNodeAndLinePlot (x1_J1, u_vectors_J1, 'ro', 5, 'r', 1.4, plt)
plt = addNodeAndLinePlot (x1_J2, u_vectors_J2, 'ro', 5, 'r', 1.4, plt)
plt = addNodePlot (x2_J1, 'bo', '', 5, plt)
plt = addNodePlot (x2_J2, 'bo', '', 5, plt)

plt.show()


