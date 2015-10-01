#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem. The main DIFFERENCE here is that the plot is always returned so that 
# plots can be added later in the Python interface.
# Use has to call himself "plt.show()"

#import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

dt = 0.02 # global drawing step size


# --------------------------------------------------------------------#

def planarPlot (cl, nPath0, nPath1, plt, lim):
    plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='g')) # Plot red cylinder
    plt.gcf().gca().add_artist(plt.Circle((-3,2),1,color='g')) # cylinder 2 (optional)
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    for t in np.arange(0., cl.problem.pathLength(nPath1), dt):
        plt.plot([cl.problem.configAtParam(nPath1, t)[0], \
                     cl.problem.configAtParam(nPath1, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath1, t)[1], \
                     cl.problem.configAtParam(nPath1, t+dt)[1]], 'k', linewidth=1.8)
    
    for t in np.arange(0., cl.problem.pathLength(nPath0), dt):
        plt.plot([cl.problem.configAtParam(nPath0, t)[0], \
                 cl.problem.configAtParam(nPath0, t+dt)[0]], \
                 [cl.problem.configAtParam(nPath0, t)[1], \
                 cl.problem.configAtParam(nPath0, t+dt)[1]], 'r')
    
    #plt.legend()
    plt.axis([-lim, lim, -lim, lim])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); #plt.grid()
    plt.plot(init[0], init[1], 'go')
    plt.plot(goal[0], goal[1], 'go')
    #plt.text(init[0]+.1, init[1]+.1, r'q_init', fontsize=11)
    #plt.text(goal[0]+.1, goal[1]+.1, r'q_end', fontsize=11)
    return plt

# --------------------------------------------------------------------#

def planarBoxPlot (cl, nPath0, nPath1, plt, lim):
    plt = plotRectangles_comb (plt)
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    for t in np.arange(0., cl.problem.pathLength(nPath1), dt):
        plt.plot([cl.problem.configAtParam(nPath1, t)[0], \
                     cl.problem.configAtParam(nPath1, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath1, t)[1], \
                     cl.problem.configAtParam(nPath1, t+dt)[1]], 'k', linewidth=2)
    
    for t in np.arange(0., cl.problem.pathLength(nPath0), dt):
        plt.plot([cl.problem.configAtParam(nPath0, t)[0], \
                 cl.problem.configAtParam(nPath0, t+dt)[0]], \
                 [cl.problem.configAtParam(nPath0, t)[1], \
                 cl.problem.configAtParam(nPath0, t+dt)[1]], 'r')
    
    plt.axis([-lim, lim, -lim, lim])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory');
    plt.plot(init[0], init[1], 'go')
    plt.plot(goal[0], goal[1], 'go')
    #plt.text(init[0]+.1, init[1]+.1, r'q_init', fontsize=11)
    #plt.text(goal[0]+.1, goal[1]+.1, r'q_end', fontsize=11)
    return plt

# --------------------------------------------------------------------#

# Plot 2D nodes (from parseLog) with given color and text.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addNodePlot (nodeList, nodeColor, nodeName, markerSize, plt):
    i = 0
    for n in nodeList :
        plt.plot(n[0], n[1], nodeColor,markersize=markerSize)
        #plt.text(n[0]+.02, n[1], nodeName+'%i' %(i), fontsize=8)
        i = i+1
    return plt

# --------------------------------------------------------------------#

# Plot 2D circles (from parseLog) with given color and size (to respect robot's size)
def addCircleNodePlot (nodeList, nodeColor, nodeSize, plt):
    i = 0
    for n in nodeList :
        plt.gcf().gca().add_artist(plt.Circle((n[0],n[1]),nodeSize,color=nodeColor))
        i = i+1
    return plt



# --------------------------------------------------------------------#

# Plot 2D trajectory (parsed from parseLog), with no label.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addPathPlot (cl, path, pathColor, lw, plt):
    size = len(path) # number of lines = 2*nbSegments
    print "pathSize= "+str(size)
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    i = 0
    while(i < size-1):
        # plot segment
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], pathColor, linewidth=lw) 
        i = i+1 # go to next segment
    # Add first and last segment
    plt.plot([init[0], path[0][0]], [init[1], path[0][1]], pathColor, linewidth=lw)
    plt.plot([goal[0], path[size-1][0]], [goal[1], path[size-1][1]], pathColor, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D path from problem-solver.
def addCorbaPathPlot (cl, nPath, pathColor, plt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        plt.plot([cl.problem.configAtParam(nPath, t)[0], cl.problem.configAtParam(nPath, t+dt)[0]], [cl.problem.configAtParam(nPath, t)[1], cl.problem.configAtParam(nPath, t+dt)[1]], pathColor, linewidth=1.5)
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for normal comb (no broken tooth)
def plotRectangles_comb (plt):
    dl = 0.05
    plt.gcf().gca().add_artist(plt.Rectangle((-1-dl/2,-0.6-2.8/2),dl,2.8,color='g')) # left big
    plt.gcf().gca().add_artist(plt.Rectangle((1-dl/2,0.6-2.8/2),dl,2.8,color='g')) # right big
    plt.gcf().gca().add_artist(plt.Rectangle((0-2/2,2-dl/2),2,dl,color='g')) # upper
    plt.gcf().gca().add_artist(plt.Rectangle((0-2/2,-2-dl/2),2,dl,color='g')) # lower
    plt.gcf().gca().add_artist(plt.Rectangle((-1-dl/2,1.6-0.8/2),dl,0.8,color='g')) # left small
    plt.gcf().gca().add_artist(plt.Rectangle((1-dl/2,-1.6-0.8/2),dl,0.8,color='g')) # right small
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for parabola environment
def plotRectangles_parab (plt, obstacles):
    lw = 1.4 # linewidth
    obstColor = 'r'
    for i in np.arange(0, len(obstacles)-1, 1):
        plt.plot([obstacles[i][0], obstacles[i+1][0]], [obstacles[i][1], obstacles[i+1][1]], obstColor, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D nodes (from parseLog) with given color, text and size.
# Plot also lines given in a list and associated to nodes. (e.g. 'x1_J1' nodes and 'u' lines)
def addNodeAndLinePlot (nodeList, lineList, nodeColor, markerSize, lineColor, lw, plt):
    i = 0
    for i in range(0,len(nodeList)):
        n = nodeList[i]
        u = lineList[i]
        plt.plot(n[0], n[1], nodeColor,markersize=markerSize)
        plt.plot([n[0], n[0]+u[0]], [n[1], n[1]+u[1]], lineColor, linewidth=lw)
        i = i+1
    return plt
