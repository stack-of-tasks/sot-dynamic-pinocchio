# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-dynamic-pinocchio.
# sot-dynamic-pinocchio is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dynamic-pinocchio is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dynamic-pinocchio. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME sot-dynamic-pinocchio)

SET(PROJECT_DESCRIPTION "pinocchio bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-dynamic-pinocchio/")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(DOXYGEN_USE_MATHJAX YES)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()
# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("eigenpy")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.6")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-tools >= 2.0.0")

SET(BOOST_COMPONENTS filesystem system unit_test_framework)

OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
ENDIF(BUILD_PYTHON_INTERFACE)


# List plug-ins that will be compiled.
SET(plugins
  zmpreffromcom
  force-compensation
  integrator-force-exact
  mass-apparent
  integrator-force-rk4
  integrator-force
  angle-estimator
  waist-attitude-from-sensor
  zmp-from-forces
  )

SET(LIBRARY_NAME ${PROJECT_NAME})
LIST(APPEND plugins dynamic)

# Add dependency toward sot-dynamic-pinocchio library in pkg-config file.
PKG_CONFIG_APPEND_LIBS(${LIBRARY_NAME})

SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
