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

SET(PROJECT_NAME sot-dynamic)
SET(PROJECT_VERSION 1.0.0.99)
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
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")

ADD_REQUIRED_DEPENDENCY("dynamicsJRLJapan >= 1.16.1")
ADD_REQUIRED_DEPENDENCY("hrp2Dynamics >= 1.3.0")
ADD_REQUIRED_DEPENDENCY("hrp2-10-optimized >= 1.0")

ADD_REQUIRED_DEPENDENCY("hrp2_10")
ADD_REQUIRED_DEPENDENCY("hrp2_10-small")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0.0")


SET(libs
	zmpreffromcom
	force-compensation
	integrator-force-exact
	mass-apparent
	integrator-force-rk4
	integrator-force
	angle-estimator
	waist-attitude-from-sensor
)

# hrp2-14 dependencies.
LIST(APPEND libs dynamic)
LIST(APPEND libs dynamic-hrp2)
SET (dynamic-hrp2_plugins_dependencies dynamic)

# hrp2-10 dependencies.
LIST(APPEND libs dynamic-hrp2_10)
SET (dynamic-hrp2_10_plugins_dependencies dynamic)
SET(dynamic-hrp2_10_plugins_compile_flags
  ${_dynamicsJRLJapan_CFLAGS} ${_hrp210optimized_CFLAGS})
SET(dynamic-hrp2_10_plugins_link_flags
  ${_dynamicsJRLJapan_LDFLAGS} ${_hrp210optimized_LDFLAGS})

LIST(APPEND libs dynamic-hrp2_10_old)
SET (dynamic-hrp2_10_plugins_dependencies dynamic)
SET(dynamic-hrp2_10_old_plugins_compile_flags
  ${_dynamicsJRLJapan_CFLAGS} ${_hrp210optimized_CFLAGS})
SET(dynamic-hrp2_10_old_plugins_link_flags
  ${_dynamicsJRLJapan_LDFLAGS} ${_hrp210optimized_LDFLAGS})


ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
