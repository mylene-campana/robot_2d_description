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
q1 = [-2, 0]; q2 = [-0.1, 2]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-0.1, 2]; q2 = [0.1, 2]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [0.1, 2]; q2 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
ps.resetGoalConfigs ()
q1 = [-2, 0]; q2 = [2, 0]
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()
# pp(3) = p0 final

ps.addPathOptimizer("GradientBased")
ps.optimizePath(3) # pp(4) = p1 final

ps.pathLength(3)
ps.pathLength(4)
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

#plt = addNodePlot ([[-0.0704869758025401, 0.03483902858869], [0.0694026091630802, 0.03736485618806798]], 'ro', 'qCol', plt)
