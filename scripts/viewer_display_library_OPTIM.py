#/usr/bin/env python
# Script which goes with scripts that test path-optimization.
# Functions library to display graphic things (trajectory, cone, frame...) on the Gepetto-viewer

from __future__ import division
import numpy as np
import math
#from hpp.gepetto import Viewer, PathPlayer

Pi = math.pi

# --------------------------------------------------------------------#

# Normalize the dir part of the configuration (dir = direction of cone)
## Parameters:
# q: given configuration
# robot: the robot
def normalizeDir (q, robot):
    q_out = q[::] # copy
    index = robot.getConfigSize () - 3
    N_norm = math.sqrt (q [index]**2 + q [index+1]**2 + q [index+2]**2)
    q_out [index] = q [index]/N_norm
    q_out [index+1] = q [index+1]/N_norm
    q_out [index+2] = q [index+2]/N_norm
    return q_out;

# --------------------------------------------------------------------#

## Plot whole path in viewer (in blue) ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# lineNamePrefix: string prefix used for line name
# dt: step time
def plotPath (cl, nPath, r, lineNamePrefix, dt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        lineName = lineNamePrefix+str(t)
        r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],cl.problem.configAtParam(nPath, t)[2]],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],cl.problem.configAtParam(nPath, t+dt)[2]],[0,0.3,1,1])
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot global frame (in white, x darker) ##
## Parameters:
# r: viewer server
# lineNameSuffix: string suffix used for line name, allowing to plot several frames
# framePosition: the [x, y, z] absolute position of the frame
# ampl: amplitude of the frame axis
def plotFrame (r, lineNameSuffix, framePosition, ampl):
    x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
    r.client.gui.addLine("frame1"+lineNameSuffix,[x,y,z], [x+ampl,y,z],[1,0,0,1])
    r.client.gui.addToGroup ("frame1"+lineNameSuffix, r.sceneName)
    r.client.gui.addLine("frame2"+lineNameSuffix,[x,y,z], [x,y+ampl,z],[0,1,0,1])
    r.client.gui.addToGroup ("frame2"+lineNameSuffix, r.sceneName)
    r.client.gui.addLine("frame3"+lineNameSuffix,[x,y,z], [x,y,z+ampl],[0,0,1,1])
    r.client.gui.addToGroup ("frame3"+lineNameSuffix, r.sceneName)

# --------------------------------------------------------------------#

## Plot straight line ##
# Uses: plot cone direction or cone - plane_theta intersection
## Parameters:
# vector: direction vector
# q: current configuration (origin of straight line)
# r: viewer server
# lineNamePrefix: string prefix used for line name
def plotStraightLine (vector, q, r, lineNamePrefix):
    x0 = q[0]
    y0 = q[1]
    z0 = q[2]
    x = vector[0]
    y = vector[1]
    z = vector[2]
    lineName=lineNamePrefix+"straight"
    r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z],[1,0.3,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot 3D points and lines between them ##
# Take two lists of points to plots them as spheres as well as lines between them
## Parameters:
# 
def plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, pointsList1, pointsList2, sphereSize):
    for i in range(0,len(pointsList1)):
        sphereName = sphereNamePrefix+str(i)
        r.client.gui.addSphere (sphereName,sphereSize,[0.7,0.5,0.3,1])
        r.client.gui.applyConfiguration (sphereName,pointsList1[i].tolist())
        r.client.gui.addToGroup (sphereName, r.sceneName)
        
        sphereName = sphereNamePrefix+str(i)+"x2"
        r.client.gui.addSphere (sphereName,sphereSize,[0.3,0.5,0.7,1])
        r.client.gui.applyConfiguration (sphereName,pointsList2[i].tolist())
        r.client.gui.addToGroup (sphereName, r.sceneName)
        
        lineName = lineNamePrefix+str(i)
        r.client.gui.addLine(lineName,[pointsList1[i][0],pointsList1[i][1],pointsList1[i][2]],[pointsList2[i][0],pointsList2[i][1],pointsList2[i][2]],[1,0.3,0,1])
        r.client.gui.addToGroup (lineName, r.sceneName)
    r.client.gui.refresh ()


# --------------------------------------------------------------------#

## Plot 3D points and lines between them ##
# Take two lists of points to plots them as spheres as well as lines between them
## Parameters:
# r: viewer server
# sphereNamePrefix: prefix for the sphere node name
# pointsList: input list of points
# sphereSize: size of displayed spheres
def plotPoints (r, sphereNamePrefix, pointsList, sphereSize):
    for i in range(0,len(pointsList)):
        sphereName = sphereNamePrefix+str(i)
        r.client.gui.addSphere (sphereName,sphereSize,[0.5,0.5,0.5,1]) # grey
        print pointsList[i].tolist()
        r.client.gui.applyConfiguration (sphereName, pointsList[i].tolist())
        r.client.gui.addToGroup (sphereName, r.sceneName)
    r.client.gui.refresh ()


# --------------------------------------------------------------------#

## Transform points in configuration ##
# Take a list of 3D points and add to each of them an identity-rotational-part
## Parameters:
# inList: input list
def transformInConfig (inList):
    outList = []
    for eachpoint in inList:
        outList.append(np.concatenate((eachpoint,[1,0,0,0]),axis=0))
    return outList # array


# --------------------------------------------------------------------#

## Plot robot base 2D path ##
# Plot the robot base 2D path with the viewer 'curve' function
## Parameters:
# r: viewer server
# cl: corbaserver client
# dt: time step to sample the displayed path
# nPath: number of 
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plot2DBaseCurvPath (r, cl, dt, nPath, pathName, pathColor):
    points = []
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        points.append ([cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0])
    
    r.client.gui.addCurve (pathName, points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)


# --------------------------------------------------------------------#

## Plot path for given DOF ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# cl: corbaserver client
# robot: robot instance
# dt: time step to sample the displayed path
# nPath: number of path on which configurations will be computed
# jointName: name of joint whose center will be displayed
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plotDofCurvPath (r, cl, robot, dt, nPath, jointName, pathName, pathColor):
    points = []
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        config = cl.problem.configAtParam(nPath, t)
        robot.isConfigValid (config) # set current config
        jointPosition = robot.getJointPosition (jointName)
        points.append ([jointPosition[0],jointPosition[1],jointPosition[2]])
    
    r.client.gui.addCurve (pathName,points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)

# --------------------------------------------------------------------#

## Plot path for given point in body DOF ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# cl: corbaserver client
# robot: robot instance
# dt: time step to sample the displayed path
# nPath: number of path on which configurations will be computed
# jointName: name of joint whose center will be displayed
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plotPointBodyCurvPath (r, cl, robot, dt, nPath, jointName, pointInJoint, pathName, pathColor):
    points = []
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        config = cl.problem.configAtParam(nPath, t)
        robot.setCurrentConfig (config)
        jointPosition = robot.getJointPosition (jointName)
        pointPosition = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)
        #print pointPosition
        points.append ([pointPosition[0],pointPosition[1],pointPosition[2]])
    
    r.client.gui.addCurve (pathName,points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)

# --------------------------------------------------------------------#

## Plot path for given body ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# cl: corbaserver client
# robot: robot instance
# dt: time step to sample the displayed path
# nPath: number of path on which configurations will be computed
# bodyName: name of body whose center will be displayed (do not forget '_0')
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plotBodyCurvPath (r, cl, robot, dt, nPath, bodyName, pathName, pathColor):
    points = []
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        config = cl.problem.configAtParam(nPath, t)
        robot.setCurrentConfig (config)
        bodyPosition = robot.getObjectPosition (bodyName)
        print bodyPosition
        points.append ([bodyPosition[0],bodyPosition[1],bodyPosition[2]])
    
    r.client.gui.addCurve (pathName,points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)



