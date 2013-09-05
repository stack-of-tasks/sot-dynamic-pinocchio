# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-dynamic.
# sot-dynamic is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dynamic is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dynamic. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-dynamic)

SET(PROJECT_DESCRIPTION "jrl-dynamics bindings for dynamic-graph.")
SET(PROJECT_URL "http://github.com/jrl-umi3218/sot-dynamic")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")

ADD_REQUIRED_DEPENDENCY("jrl-dynamics >= 1.19.0")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 2.5.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 2.5")
ADD_REQUIRED_DEPENDENCY("sot-tools")

# List plug-ins that will be compiled.
SET(libs
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

LIST(APPEND libs dynamic)


LIST(APPEND LOGGING_WATCHED_TARGETS ${libs})

# Add dependency toward sot-dynamic library in pkg-config file.
PKG_CONFIG_APPEND_LIBS_RAW(${DYNAMIC_GRAPH_PLUGINDIR}/dynamic.so)
PKG_CONFIG_APPEND_LIBRARY_DIR(${DYNAMIC_GRAPH_PLUGINDIR})

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem system)
SEARCH_FOR_BOOST()

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
