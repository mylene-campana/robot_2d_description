#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and spiral-waypoints.

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

# q = [x, y] # limits in URDF
q1 = [0, 0]; q2 = [-0.5, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-0.5, 0.5]; q2 = [0, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0, 1]; q2 = [1, 0]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [1, 0]; q2 = [0, -1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0, -1]; q2 = [-1.5, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-1.5, 0.5]; q2 = [0, 2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0, 2]; q2 = [2, 0]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [2, 0]; q2 = [0, -2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0, 0]; q2 = [0, -2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
# pp(8) = p0 final
cl.problem.optimizePath(9) # pp(9) = p1 final


begin=time.time()
cl.problem.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
cl.problem.pathLength(0)
cl.problem.pathLength(1)

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 10700
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlotCylinder, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:327: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:321: qColl = ')

x0Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:284: x0=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:286: finish path parsing',2,0)
x1Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:289: x0+alpha*p -> x1=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:291: finish path parsing',2,0)
x2Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:289: x0+alpha*p -> x1=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:291: finish path parsing',3,0)
x3Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:289: x0+alpha*p -> x1=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:291: finish path parsing',4,0)
x4Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:289: x0+alpha*p -> x1=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:291: finish path parsing',5,0)
x5Path = parsePathVector (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:289: x0+alpha*p -> x1=','INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:291: finish path parsing',6,0)

plt = planarPlotCylinder (cl, 8, 9, plt) # initialize 2D plot with obstacles and path
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 1, 'qCol', plt)
plt = addPathPlot (cl, x0Path, 'm', 1, plt)
plt = addPathPlot (cl, x1Path, 'g', 1, plt)
plt = addPathPlot (cl, x2Path, 'b', 1, plt)
plt = addPathPlot (cl, x3Path, 'y', 1, plt)
plt = addPathPlot (cl, x4Path, 'c', 1, plt)
plt = addPathPlot (cl, x5Path, '0.75', 1, plt)
plt.show() # will reset plt


## Viewer ##
from hpp.gepetto import Viewer, PathPlayer
Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (cl, r)

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

