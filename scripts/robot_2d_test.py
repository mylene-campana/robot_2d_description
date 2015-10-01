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
q1 = [-2, 0]; q2 = [-0.2, 2]; q3 = [0.2, 2]; q4 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4); ps.solve (); ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q4); ps.solve (); ps.resetGoalConfigs ()
# pp(3) = p0 final

#ps.addPathOptimizer("GradientBased")
#ps.addPathOptimizer("Prune")
ps.addPathOptimizer("PartialRandomShortcut")
ps.optimizePath(3) # pp(4) = p1 final

ps.pathLength(3)
ps.pathLength(4)
ps.getWaypoints (3)
ps.getWaypoints (4)
# should be [-0.07 0] [0.07 0] if alpha_init=1


"""
q1 = [-2, 0]; q2 = [-1, 1]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-1, 1]; q2 = [-1.2, 1.8]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-1.2, 1.8]; q2 = [-0.2, 1.2]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-0.2, 1.2]; q2 = [0.5, 1.9]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [0.5, 1.9]; q2 = [2, 1.5]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [2, 1.5]; q2 = [1, 0.5]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [1, 0.5]; q2 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-2, 0]; q2 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
# pp(7) = p0 final
ps.optimizePath(7) # pp(8) = p1 final

ps.pathLength(7)
ps.pathLength(8)
"""

len(ps.nodes ())
len(ps.getWaypoints (0))
cl.problem.getIterationNumber()

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
nPath = 4
lineNamePrefix = "optimized_pathi"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0.1],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0.1],[0,1,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

nPath = 3
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

