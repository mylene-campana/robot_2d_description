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

# q = [x, y] # limits in URDF file
"""
q1 = [-2, 0]; q2 = [-0.2, 2]; q3 = [0.2, 2]; q4 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q4); ps.solve (); #3
"""

q1 = [-2, 0]; q2 = [-1, 1]; q3 = [-1.2, 1.8]; q4 = [-0.2, 1.2];  q5 = [0.5, 1.9]
q6 = [2, 1.5]; q7 = [1, 0.5]; q8 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q4); ps.addGoalConfig (q5); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q5); ps.addGoalConfig (q6); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q6); ps.addGoalConfig (q7); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q7); ps.addGoalConfig (q8); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q8); ps.solve (); # 7

ps.addPathOptimizer("GradientBased")
#ps.addPathOptimizer("Prune")
#ps.addPathOptimizer("PartialRandomShortcut")
ps.optimizePath(7)

ps.pathLength(7)
ps.pathLength(8)

ps.getWaypoints (7)
ps.getWaypoints (8)



"""
from hpp.gepetto import Viewer, PathPlayer
Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")
"""

git commit -m "Remove inverse Hessian computation from Cost"
## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 12640
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:317: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:307: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:86: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:89: finish path parsing'
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, 0)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, 0)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, 0)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, 0)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, 0)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, 0)

#plt = planarPlot (cl, 7, 8, plt, 2) # initialize 2D plot with obstacles and path
plt = planarPlot (cl, 3, 4, plt, 2)
plt = addPathPlot (cl, x0Path, 'm', 1, plt)
plt = addPathPlot (cl, x1Path, 'g', 1, plt)
plt = addPathPlot (cl, x2Path, 'b', 1, plt)
plt = addPathPlot (cl, x3Path, 'y', 1, plt)
plt = addPathPlot (cl, x4Path, 'c', 1, plt)
plt = addPathPlot (cl, x5Path, '0.75', 1, plt)
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt = addNodePlot ([ps.getWaypoints (4)[1], ps.getWaypoints (4)[2]], 'go', 'qCol', plt)
plt.show() # will reset plt

#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( ps.configAtParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")

# plot paths in viewer
import numpy as np
dt = 0.1
nPath = 8
lineNamePrefix = "optimized_pathghj"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0.1],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0.1],[0,1,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

nPath = 7
lineNamePrefix = "initial_path"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0],[1,0.3,0,1])
    r.client.gui.addToGroup (lineName, r.sceneName)


nPath = 3
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve ("initCurvPathc",points,[1,0.3,0,1])
r.client.gui.addToGroup ("initCurvPathc", r.sceneName)


qColl = [-0.107713,0.627621]
qFree = [-0.10918,0.697357]
contactPoint = [-0.0843029,0.491217,0,1,0,0,0] # x2_J2
x1_J2 = [-0.0857704,0.560953,0,1,0,0,0]
u = [-0.00146755,0.0697357,0,0,0,0,0]

uLine1 = +
uLine2 = contactPoint

sphereName = "contactPointSpheretsd"
r.client.gui.addSphere (sphereName,0.015,[0.7,0.5,0.3,1])
r.client.gui.applyConfiguration (sphereName,x1_J2)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()
sphereName = "contactPointSpheredrhy"
r.client.gui.addSphere (sphereName,0.015,[0.7,0.5,0.3,1])
r.client.gui.applyConfiguration (sphereName,contactPoint)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

r.client.gui.removeFromGroup ("initCurvPath", r.sceneName)

lineName = "u2"
uLine = np.array(x1_J2)+np.array(u)
r.client.gui.addLine(lineName,[uLine[0],uLine[1],uLine[2]],[contactPoint[0],contactPoint[1],contactPoint[2]],[1,0.3,0,1])
r.client.gui.addToGroup (lineName, r.sceneName)


import math
wps = ps.getWaypoints(4)
dist = 0
for i in range(0,len(wps)-1):
    wpi = wps [i]
    wpii = wps [i+1]
    dist += math.sqrt( (wpii [0] - wpi [0])**2 +  (wpii [1] - wpi [1])**2  )
    print dist


## Debug Optimization Tools ##############
import matplotlib.pyplot as plt
num_log = 31069
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

plt = planarPlot (cl, 7, ps.numberPaths()-1, plt, 2.1)
i=0
for i in range(8,ps.numberPaths()-1):
    plt = addCorbaPathPlot (cl, i, '0.75', plt)

plt = addCircleNodePlot (collConstrNodes, 'b', 0.14, plt)
#plt = addCircleNodePlot (collNodes, 'r', 0.14, plt)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt = addNodeAndLinePlot (x1_J1, u_vectors_J1, 'ro', 5, 'r', 1.4, plt)
plt = addNodeAndLinePlot (x1_J2, u_vectors_J2, 'ro', 5, 'r', 1.4, plt)
plt = addNodePlot (x2_J1, 'bo', '', 5, plt)
plt = addNodePlot (x2_J2, 'bo', '', 5, plt)

plt = addNodePlot (collConstrNodes, 'ko', '', 5, plt)

plt.show()


