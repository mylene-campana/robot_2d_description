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
cl.obstacle.loadObstacleModel('robot_2d_description','cylinder_obstacle2','')

"""
from hpp.gepetto import Viewer, PathPlayer
Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle","cylinder_obstacle")
r.loadObstacleModel ("robot_2d_description","cylinder_obstacle2","cylinder_obstacle2")
"""

# q = [x, y] # limits in URDF file
q1 = [-7, 3]; q2 = [-8, 2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-8, 2]; q2 = [-6, 2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-6, 2]; q2 = [-7, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-7, 1]; q2 = [-5, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-5, -1]; q2 = [-8, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-8, -1]; q2 = [-4, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-4, 1]; q2 = [-2, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-2, -1]; q2 = [0.2, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0.2, 1]; q2 = [1, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [1, -1]; q2 = [4, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [4, 1]; q2 = [5, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-7, 3]; q2 = [5, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
# pp(11) = p0 final
begin=time.time()
cl.problem.optimizePath(11) # pp(12) = p1 final
end=time.time()
print "Solving time: "+str(end-begin)


len(cl.problem.nodes ())
len(ps.getWaypoints (0))
cl.problem.pathLength(11)
cl.problem.pathLength(12)
cl.problem.getIterationNumber()

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 10081
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:191: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:185: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:147: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:149: finish path parsing'
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

