#!/usr/bin/env python
# Copyright (c) 2015 CNRS
# Author: Mylene Campana
#
# This file is part of robot_2d_description.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# robot_2d_description is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Call urdf description of cylinder planar robot.

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
    packageName = "robot_2d_description"
    #urdfName = "object" # (0.05 radius)
    urdfName = "large_object" # (0.14 radius)
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__ (self, robotName):
        Parent.__init__ (self, robotName, "anchor")
        self.tf_root = "base_link"
        # same as the urdf root name
