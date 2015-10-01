#/usr/bin/env python
# Parser script to gather float vector from a (Log) file
import numpy as np

logFile = "/local/mcampana/devel/hpp/install/var/log/hpp/"
endWhile = "INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:37: finalPath:"

# Parse configurations of given path, until the endWhile line is reached.
def parseConfig (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        configs = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip (',\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    config = map (float, sp) # convert into float
                    configs.append (config)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*configs)) # transpose and make array

"""
gradAtt :
_,    _,
_,  , _,  , ....
_,    _,
"""

# --------------------------------------------------------------------#

# Depend on how is written the vector in Log file.
# Easier if separated by comas than irregular spaces.
def parseGrad (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        grads = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip ('\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    grad = map (float, sp) # convert into float
                    grads.append (grad)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*grads)) # transpose and make array


# --------------------------------------------------------------------#

# Parse configurations without end condition. 
# Used to parse guard nodes of visibility-prm algorithm or configuration in
# Gradient-based optimization.
def parseNodes (pid, prefix):
    prefixFile = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/collision-constraints-result.hh:'
    l = len (prefixFile+prefix)
    lend = len(endWhile)
    with open (logFile + "journal." + str(pid) + ".log") as f:
        nodes = []
        for line in f:
            if line [:l] == prefixFile+prefix:
                suffix = line [l:]
                st = suffix.strip (',\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    node = map (float, sp) # convert into float
                    nodes.append (node)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
    #return np.array (zip (*nodes)) # transpose and make array
    return np.array (nodes) # make array

# --------------------------------------------------------------------#

# Parse a pathVector returned in a hpp log file, until the endLine is reached.
# iterNB allows to select the optim iteration which we want to plot.
def parsePathVector (pid, beginLine, endLine, iterNB, skipLines):
    prefixConfig = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/include/hpp/core/path-optimization/gradient-based.hh:82: '
    l = len(prefixConfig)
    lineNB = 0
    itNB = 1 # current iteration number (of optim algo)
    confVect = [0, 0] # 2D row
    with open (logFile + "journal." + str(pid) + ".log") as f:
        lines=f.readlines()
        configs = []
        for line in lines:
            lineNB = lineNB+1
            if (line.strip('\n') == beginLine) : # begin line found
                if (itNB == iterNB): # wanted iteration
                    #lineNB = lineNB + 1 # Start parsing path until the final line is reached
                    while (1):
                        actualLine = lines[lineNB]
                        actualLine1 = lines[lineNB+1]
                        suffix = actualLine[len (prefixConfig):]
                        suffix1 = actualLine1[len (prefixConfig):]
                        st = suffix.strip ('\n') # remove end characters
                        st1 = suffix1.strip ('\n')
                        #print float(st)
                        confVect[0] = float(suffix)
                        confVect[1] = float(suffix1)
                        configs.append(confVect[:])
                        print confVect
                        # skip 2 useless lines + skipLines others (ex: 2 for plannar quatern)
                        lineNB = lineNB + skipLines + 2 
                        if (lines [lineNB].strip ('\n') == endLine):
                            #print "END OF PARSING"
                            return np.array (configs) # make array and finish
                itNB = itNB+1

# --------------------------------------------------------------------#

# Parse parabola information contained in a hpp log
def parseParabola (pid, parabNB):
    prefixConfig = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-path.cc:65: entering virtual bool hpp::core::ParabolaPath::impl_compute(hpp::core::ConfigurationOut_t, hpp::core::value_type) const'
    prefixX = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-path.cc:65: x: '
    prefixFX = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/parabola/parabola-path.cc:66: f(x): '
    endLine = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/discretized-collision-checking.cc:112: exiting virtual bool hpp::core::ParabolaPath::impl_compute(hpp::core::ConfigurationOut_t, hpp::core::value_type) const'
    endLine2 = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-planner.cc:141: exiting virtual bool hpp::core::ParabolaPath::impl_compute(hpp::core::ConfigurationOut_t, hpp::core::value_type) const'
    endLine3 = 'INFO:/local/mcampana/devel/hpp/install/include/hpp/core/basic-configuration-shooter.hh:61: exiting virtual bool hpp::core::ParabolaPath::impl_compute(hpp::core::ConfigurationOut_t, hpp::core::value_type) const'
    
    lineNB = 0
    itNB = 1 # parabola number
    confVect = [0, 0] # 2D row
    with open (logFile + "journal." + str(pid) + ".log") as f:
        lines=f.readlines()
        configs = []
        for line in lines:
            lineNB = lineNB+1
            if line [:len(prefixConfig)] == prefixConfig:
                if (itNB == parabNB): # wanted parabola
                    #lineNB = lineNB + 1 # Start parsing path until the final line is reached
                    while (1):
                        actualLine = lines[lineNB]
                        print actualLine
                        actualLine1 = lines[lineNB+1]
                        print actualLine1
                        suffix = actualLine[len (prefixX):]
                        suffix1 = actualLine1[len (prefixFX):]
                        st = suffix.strip ('\n') # remove end characters
                        st1 = suffix1.strip ('\n')
                        confVect[0] = float(suffix)
                        confVect[1] = float(suffix1)
                        configs.append(confVect[:])
                        print confVect
                        lineNB = lineNB + 2 + 1 # skip gamma line
                        print lines [lineNB].strip ('\n')
                        if (lines [lineNB].strip ('\n') == endLine) or (lines [lineNB].strip ('\n') == endLine2) or (lines [lineNB].strip ('\n') == endLine3):
                            print "END OF PARSING"
                            return np.array (configs) # make array and finish
                itNB = itNB+1

# --------------------------------------------------------------------#

# Parse 3D points for collision-constraint visualization
# points are defined as: (x y z)
def parseCollConstrPoints (pid, prefix):
    prefixFile = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/collision-constraints-result.hh:'
    
    l = len (prefixFile+prefix)
    with open (logFile + "journal." + str(pid) + ".log") as f:
        points = []
        for line in f:
            if line [:l] == prefixFile+prefix:
                suffix = line [l:]
                st = suffix.strip (')\n') # remove end characters
                sp = st.split (' ') # separate numbers with coma
                try:
                    point = map (float, sp) # convert into float
                    points.append (point)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
    #return np.array (zip (*nodes)) # transpose and make array
    return np.array (points) # make array

