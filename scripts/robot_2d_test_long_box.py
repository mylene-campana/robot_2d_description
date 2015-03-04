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
cl.obstacle.loadObstacleModel('robot_2d_description','box','')

"""
from hpp.gepetto import Viewer, PathPlayer
Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("robot_2d_description","box","box")
"""

# q = [x, y] # limits in URDF file
dist=100
q1 = [-dist, 1]; q2 = [-0.8, 1]; q3 = [0.5, 1.5]; q4 = [0.7, 0]; q5 = [-0.7, -1.7];
q6 = [0.4,-1.5]; q7 = [-0.6, -0.2]; q8 = [0.2, 0.2]; q9 = [1, -1]; q10 = [dist, -1]; 

cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q2); cl.problem.addGoalConfig (q3)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q3); cl.problem.addGoalConfig (q4)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q4); cl.problem.addGoalConfig (q5)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q5); cl.problem.addGoalConfig (q6)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q6); cl.problem.addGoalConfig (q7)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q7); cl.problem.addGoalConfig (q8)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q8); cl.problem.addGoalConfig (q9)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q9); cl.problem.addGoalConfig (q10)
cl.problem.solve (); cl.problem.resetGoalConfigs ()
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q10); cl.problem.solve ();


# pp(9) = p0 final
begin=time.time()
cl.problem.optimizePath(9) # pp(10) = p1 final
end=time.time()
print "Optim time: "+str(end-begin)

#len(ps.getWaypoints (0))
cl.problem.getIterationNumber()
cl.problem.pathLength(9)
cl.problem.pathLength(10)


## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 4230
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarBoxPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:192: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:186: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:148: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:150: finish path parsing'
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

