#/usr/bin/env python
# Script which goes with potential_description package.
# Load planar 'robot' cylinder and concave obstacles.


from hpp.corbaserver.potential import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
import matplotlib.pyplot as plt
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

kRange = 5
robot = Robot ('potential')
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
ps = ProblemSolver (robot)
cl = robot.client

# q = [x, y, theta] # (z not considered since planar)
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1
#q1 = [-4, 4, 0]; q2 = [4, -4, 0] # obstS 1

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','obstacles_concaves')

#ps.createOrientationConstraint ("orConstraint", "base_joint_rz", "", [1,0,0,0], [0,0,1])
#ps.setNumericalConstraints ("constraints", ["orConstraint"])

ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.selectPathValidation ("Dichotomy", 0.)

ps.solve ()
ps.pathLength(0)

ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)


import matplotlib.pyplot as plt
from mutable_trajectory_plot import planarPlot, addNodePlot
from parseLog import parseCollConstrPoints
num_log = 31891
contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
plt = planarPlot (cl, 0, ps.numberPaths()-1, plt, 1.5, 5)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt.show()


ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)

ps.clearPathOptimizers()



len(ps.getWaypoints (0))


from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
r.loadObstacleModel ("potential_description","obstacles_concaves","obstacles_concaves")

import numpy as np
dt = 0.1
nPath = 0
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve ("initCurvPath",points,[1,0.3,0,1])
r.client.gui.addToGroup ("initCurvPath", r.sceneName)

nPath = 1
points = []
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])

r.client.gui.addCurve ("optimCurvPathc",points,[0,0.3,1,1])
r.client.gui.addToGroup ("optimCurvPathc", r.sceneName)

r.client.gui.removeFromGroup ("optimCurvPath", r.sceneName)


#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2
#cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle','')

## Debug Optimization Tools ##############

import matplotlib.pyplot as plt
num_log = 12903
from parseLog import parseNodes, parsePathVector
from mutable_trajectory_plot import planarPlot, addNodePlot, addPathPlot

collConstrNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:318: qCollConstr = ')
collNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:308: qColl = ')

x1initLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:86: x0+alpha*p -> x1='
x1finishLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:89: finish path parsing'
skipLines = 1 # skip useless lines while parsing path, usually skip rotation parts
x0Path = parsePathVector (num_log, x1initLine, x1finishLine, 1, skipLines)
x1Path = parsePathVector (num_log, x1initLine, x1finishLine, 2, skipLines)
x2Path = parsePathVector (num_log, x1initLine, x1finishLine, 3, skipLines)
x3Path = parsePathVector (num_log, x1initLine, x1finishLine, 4, skipLines)
x4Path = parsePathVector (num_log, x1initLine, x1finishLine, 5, skipLines)
x5Path = parsePathVector (num_log, x1initLine, x1finishLine, 6, skipLines)
x6Path = parsePathVector (num_log, x1initLine, x1finishLine, 7, skipLines)


plt = planarPlot (cl, 0, ps.numberPaths()-1, plt) # initialize 2D plot with obstacles and path
plt = addPathPlot (cl, x0Path, '0.9', 1, plt)
plt = addPathPlot (cl, x1Path, '0.8', 1, plt)
plt = addPathPlot (cl, x2Path, '0.7', 1, plt)
plt = addPathPlot (cl, x3Path, '0.6', 1, plt)
plt = addPathPlot (cl, x4Path, '0.5', 1, plt)
plt = addPathPlot (cl, x5Path, '0.4', 1, plt)
plt = addPathPlot (cl, x6Path, '0.3', 1, plt)
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt.show() # will reset plt

"""
plt = planarPlot (cl, 0, 1, plt) # initialize 2D plot with obstacles and path
plt = addNodePlot (collConstrNodes, 'bo', 'qConstr', plt)
plt = addNodePlot (collNodes, 'ro', 'qCol', plt)
plt = addPathPlot (cl, x0Path, 'm', 1, plt)
plt = addPathPlot (cl, x1Path, 'g', 1, plt)
plt = addPathPlot (cl, x2Path, 'b', 1, plt)
plt = addPathPlot (cl, x3Path, 'y', 1, plt)
plt = addPathPlot (cl, x4Path, 'c', 1, plt)
plt = addPathPlot (cl, x5Path, '0.8', 1, plt)
plt = addPathPlot (cl, x6Path, '0.5', 1, plt)
plt.show() # will reset plt
"""
#####################################################################

from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 1, '', 0) # don't plot "equirepartis" nodes


#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtParam(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot Trajectory, x, y, dist_obst #
from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

# Gradients arrows Plot on 2D plan graph (with trajectory) #
from parseLog import parseGrad, parseConfig
num_log = 10435 # TO_FILL, and UPDATE following line numbers
q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:381: q(x,y): ')
grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:383: grad(x,y): ')
q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/diffusing-planner.cc:121: q_rand = ') # diffusingPlanner
#q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: q(x,y): ')
#grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: grad(x,y): ')
#q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-planner.cc:105: q_rand = ') # superposedPlanner

from trajectory_plot import gradArrowsConcPlot # concave
gradArrowsConcPlot(cl, q_list, grad_list, q_rand_list, 0)

from trajectory_plot import gradArrowsPlot # cylinder
gradArrowsPlot(cl, q_list, grad_list, q_rand_list)

# Visibility-PRM verification :
num_log = 7770
from parseLog import parseNodes
guardNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/visibility-prm-planner.cc:176: q is a guard node: ')
from trajectory_plot import planarConcObstaclesSpecNodesPlot
planarConcObstaclesSpecNodesPlot(cl, 0, '', 0, guardNodes) # guards in blue


## Debug Optimization Tools ##############
import matplotlib.pyplot as plt
num_log = 21173
from parseLog import parseCollConstrPoints, parseNodes
from mutable_trajectory_plot import planarPlot, addNodePlot, addCircleNodePlot, addNodeAndLinePlot, addCorbaPathPlot

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '94: x1 in J1 = (')
x2_J1 = parseCollConstrPoints (num_log, '95: x2 in J1 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

plt = planarPlot (cl, 0, ps.numberPaths()-1, plt, 1.5, 5)
i=1; iMax = ps.numberPaths()-1;
while (i < iMax):
    print i
    plt = addCorbaPathPlot (cl, i, '0.75', 0.9, plt)
    i = i + 2

#plt = addCorbaPathPlot (cl, 0, 'rk', 1.6, plt)
plt = addCorbaPathPlot (cl, ps.numberPaths()-1, 'k', 1.5, plt)
plt = addCircleNodePlot (collConstrNodes, 'b', 0.02, plt)
#plt = addCircleNodePlot (collNodes, 'r', 0.02, plt)
plt = addNodePlot (contactPoints, 'ko', '', 5.5, plt)
plt = addNodeAndLinePlot (x1_J1, x2_J1, 'ro', 'bo', 4, 'r', 1.3, plt)
plt = addNodeAndLinePlot (x1_J2, x2_J2, 'ro', 'bo', 4, 'r', 1.3, plt)

#plt = addNodePlot (collConstrNodes, 'ko', '', 5, plt)

plt.show()





## Debug Optimization Tools ##############
num_log = 17880
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath, plotBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)

# Plot points
sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

# Plot trajectories
from hpp.corbaserver import Client
cl = robot.client
dt = 0.08

jointName = 'base_joint_xy'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0,0], 'pathPoint_'+str(0)+jointName, [1,0,0,1])
plotPointBodyCurvPath (r, cl, robot, dt, 1, jointName, [0,0,0], 'pathPointRS_'+str(1)+jointName, [0,0,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0,0,0], 'pathPointGB_'+str(ps.numberPaths ()-1)+jointName, [0,1,0,1])



# test function in cl.robot
jointPosition = robot.getJointPosition ('wrist_3_joint')
pointInJoint = [0.3,0,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q1)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()
