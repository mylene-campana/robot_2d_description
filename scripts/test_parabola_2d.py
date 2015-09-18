#/usr/bin/env python
# Script which goes with robot_2d_description package.

from hpp.corbaserver.robot_2d import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
#import time
#import sys
#import matplotlib.pyplot as plt
#sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('robot_2d')
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('robot_2d_description','environment_2d','')
#cl.obstacle.loadObstacleModel('robot_2d_description','environment_2d_harder','')

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (cl, r)
#r.loadObstacleModel ("robot_2d_description","environment_2d","environment_2d")
r.loadObstacleModel ("robot_2d_description","environment_2d_harder","environment_2d_harder")
#r.loadObstacleModel ("iai_maps", "labyrinth", "labyrinth")
#r.loadObstacleModel ("iai_maps", "labyrinth2", "labyrinth2") 

# q = [x, y, dir.x, dir.y] # limits in URDF file
#q1 = [-1.3, 0.2]; q2 = [0.2, 0.2]; q3 = [5, -2.8]; q4 = [9, 2.2]; q5 = [12.4, -5.8];
#q1 = [-1.3, 0.2, 0]; q2 = [0.2, 0.2, 0]; q3 = [5, -2.8, 0]; q4 = [9, 2.2, 0]; q5 = [12.4, -5.8, 0];
q1 = [-1.3, 0.2, 0, 1];  q5 = [12.4, -5.8, 0, 1];
ps.setInitialConfig (q1); ps.addGoalConfig (q5); ps.solve ()


# with sub-paths
q1 = [-1.3, 0.2, 0, 1]; q2 = [0.2, 0.2, 0, 1]; q3 = [5, -2.8, 0, 1]; q4 = [9, 2.2, 0, 1];
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve () # pp(0)
ps.resetGoalConfigs ()
ps.setInitialConfig (q2); ps.addGoalConfig (q3); ps.solve () # pp(1)
ps.resetGoalConfigs ()
ps.setInitialConfig (q3); ps.addGoalConfig (q4); ps.solve () # pp(2)
ps.resetGoalConfigs ()
ps.setInitialConfig (q4); ps.addGoalConfig (q5); ps.solve () # pp(3)
ps.resetGoalConfigs ()
ps.setInitialConfig (q1); ps.addGoalConfig (q5); ps.solve () # pp(4)
ps.resetGoalConfigs ()

ps.pathLength(0)
cl.problem.getWaypoints (0)
ps.configAtParam(0,0.2)

ps.pathLength(4)
cl.problem.getWaypoints (4)


## Plot whole path in viewer ##
import numpy as np
nPath = 0
dt = 0.1
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = "g"+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0.2],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0.2],[0,0.3,1,1])
    r.client.gui.addToGroup (lineName, r.sceneName)


## Video capture ##
r.startCapture ("capture","png")
pp(0)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


## Path drawing ##
import matplotlib.pyplot as plt
num_log = 23374
from parseLog import parseNodes, parseParabola
from parabola_plot_tools import parabPlot, addNodePlot, plotParsedParab
nPath = 0
mu = 0.5

RandConfig = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-planner.cc:187: qrand: ')
ProjNodes = parseNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-planner.cc:203: q_proj: ')

plt = addNodePlot (RandConfig, 'bo', 'qrand', plt)
plt = addNodePlot (ProjNodes, 'ro', 'qproj', plt)

plt = parabPlot (cl, nPath, mu, plt)


plt.show() # will reset plt


parab = parseParabola (num_log, 1)
plt = plotParsedParab (cl, parab, 'b', 1, plt)


#####################################################################

## DEBUG commands
robot.isConfigValid(q1)
cl.robot.distancesToCollision()
from numpy import *
cl.robot.distancesToCollision()[1][argmin(cl.robot.distancesToCollision()[0])]
cl.robot.distancesToCollision()[2][argmin(cl.robot.distancesToCollision()[0])]
cl.robot.distancesToCollision()[0][argmin(cl.robot.distancesToCollision()[0])]
r( ps.configAtParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
#cl.obstacle.getObstacleNames(True,False)

