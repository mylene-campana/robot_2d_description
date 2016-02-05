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


nInitialPath = ps.numberPaths()-1 #8
ps.pathLength(nInitialPath)

ps.addPathOptimizer("GradientBased")
#ps.addPathOptimizer("Prune")
#ps.addPathOptimizer("PartialRandomShortcut")

ps.optimizePath(nInitialPath)
ps.pathLength(ps.numberPaths()-1)

ps.getWaypoints (nInitialPath)
ps.getWaypoints (ps.numberPaths()-1)



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
num_log = 19352
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
r.client.gui.removeFromGroup (elementName, r.sceneName)

# Compute manually path length with waypoints
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
num_log = 31733
from parseLog import parseCollConstrPoints, parseNodes
from mutable_trajectory_plot import planarPlot, addNodePlot, addCircleNodePlot, addNodeAndLinePlot, addCorbaPathPlot

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '94: x1 in J1 = (')
x2_J1 = parseCollConstrPoints (num_log, '95: x2 in J1 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

plt = planarPlot (cl, nInitialPath, ps.numberPaths()-1, plt, 1.5, 2.1)

i=0
for i in range(nInitialPath+1,ps.numberPaths()-1):
    plt = addCorbaPathPlot (cl, i, '0.75', plt)

plt = addCorbaPathPlot (cl, 8, '0.5', 1, plt)

#plt = addCorbaPathPlot (cl, ps.numberPaths()-1, 'k', 1, plt)
#plt = addCircleNodePlot (collConstrNodes, 'b', 0.14, plt)
#plt = addCircleNodePlot (collNodes, 'r', 0.14, plt)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt = addNodeAndLinePlot (x1_J1, x2_J1, 'ro', 'bo', 5, 'r', 1.4, plt)
plt = addNodeAndLinePlot (x1_J2, x2_J2, 'ro', 'bo', 5, 'r', 1.4, plt)

#plt = addNodePlot (collConstrNodes, 'ko', '', 5, plt)

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
plot2DBaseCurvPath (r, cl, dt, 7, "curvPath"+str(7), [1,0.3,0,1])
plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPath"+str(ps.numberPaths()-1), [0,1,0.3,1])

