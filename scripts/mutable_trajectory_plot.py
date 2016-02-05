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

def planarPlot (cl, nPath0, nPath1, plt, lw, lim):
    #plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='g')) # Plot red cylinder
    #plt.gcf().gca().add_artist(plt.Circle((-3,2),1,color='g')) # cylinder 2 (optional)
    plt = obstacles (plt, True)
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    for t in np.arange(0., cl.problem.pathLength(nPath1), dt):
        plt.plot([cl.problem.configAtParam(nPath1, t)[0], \
                     cl.problem.configAtParam(nPath1, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath1, t)[1], \
                     cl.problem.configAtParam(nPath1, t+dt)[1]], 'k', linewidth=lw)
    
    for t in np.arange(0., cl.problem.pathLength(nPath0), dt):
        plt.plot([cl.problem.configAtParam(nPath0, t)[0], \
                 cl.problem.configAtParam(nPath0, t+dt)[0]], \
                 [cl.problem.configAtParam(nPath0, t)[1], \
                 cl.problem.configAtParam(nPath0, t+dt)[1]], 'r', linewidth=lw)
    
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
def addCorbaPathPlot (cl, nPath, pathColor, lw, plt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        plt.plot([cl.problem.configAtParam(nPath, t)[0], cl.problem.configAtParam(nPath, t+dt)[0]], [cl.problem.configAtParam(nPath, t)[1], cl.problem.configAtParam(nPath, t+dt)[1]], pathColor, linewidth=lw)
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
# Plot also lines between each combination of the two nodes lists.
def addNodeAndLinePlot (nodeList1, nodeList2, nodeColor1, nodeColor2, markerSize, lineColor, lw, plt):
    i = 0
    for i in range(0,len(nodeList1)):
        n1 = nodeList1[i]
        n2 = nodeList2[i]
        plt.plot(n1[0], n1[1], nodeColor1, markersize=markerSize)
        plt.plot(n2[0], n2[1], nodeColor2, markersize=markerSize)
        plt.plot([n1[0], n2[0]], [n1[1], n2[1]], lineColor, linewidth=lw)
        i = i+1
    return plt


# --------------------------------------------------------------------#

# Plot desired obstacles: if bool 'concave' is True plot concave obstacles, else plot cylinder
def obstacles (plt, concave):
    dr=0
    if concave:
        plt.gcf().gca().add_artist(plt.Circle((0.9,0.4),.6+dr,color='green')) # orange
        #plt.text(0.9, 0.4, r'obst1', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((1.5,0.4),.6+dr,color='green')) # orange
        #plt.text(1.5, 0.3, r'obst1bis', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2+dr,color='green'))
        #plt.text(1.2, -3.5, r'obst2', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2+dr,color='green'))
        #plt.text(2.6, 2.5, r'obst6', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6+dr,color='green'))
        #plt.text(-2.5, 1.2, r'obst4', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-1,4),.6+dr,color='green'))
        #plt.text(-1, 4, r'obst5', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2+dr,color='green'))
        #plt.text(-2.8, -2.2, r'obst3', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6+dr,color='green'))
        #plt.text(3.8, -1, r'obst7', fontsize=11)
        plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8+dr,1.8,color='green'))
        #plt.text(0, 0, r'obst_base', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((1.5,-1.8),0.6+dr,color='green'))
        #plt.text(1.5, -1.8, r'obst8', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((2., 1.2),0.4+dr,color='green')) # light_blue
        #plt.text(2., 1.2, r'obst9', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-1.6,1.4),0.5+dr,color='green')) # yellow
        #plt.text(-1.6, 1.4, r'obst10', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-1.5,2.3),0.5+dr,color='green')) # yellow
        #plt.text(-1.5, 2.3, r'obst11', fontsize=11)
        plt.gcf().gca().add_artist(plt.Circle((-0.9,3.),0.5+dr,color='green')) # yellow
        #plt.text(-0.9, 3., r'obst12', fontsize=11)
    else:
        plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='r')) # Plot red cylinder
    return plt

