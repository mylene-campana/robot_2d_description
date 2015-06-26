# Copyright (c) 2015 CNRS
# Author: Mylene Campana
#
# This file is part of robot_2d_description.
# robot_2d_description is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# robot_2d_description is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# robot_2d_description.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME robot_2d_description)
SET(PROJECT_DESCRIPTION "Data specific to 2D robot for hpp-corbaserver")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES urdf/object.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES urdf/cylinder_obstacle.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES urdf/cylinder_obstacle2.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES urdf/environment_2d.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES urdf/environment_2d_harder.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES urdf/box.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES srdf/object.srdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
  
install(FILES urdf/large_object.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES srdf/large_object.srdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install (FILES
  src/hpp/corbaserver/robot_2d/robot.py
  src/hpp/corbaserver/robot_2d/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/robot_2d)
  
SETUP_PROJECT_FINALIZE()
