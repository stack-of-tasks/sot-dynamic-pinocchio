# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)
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

INCLUDE(../cmake/python.cmake)
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

IF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

SET(integrator-force-rk4_plugins_dependencies integrator-force)
SET(integrator-force-exact_plugins_dependencies integrator-force)

SET(dynamic-hrp2_plugins_dependencies dynamic)
SET(dynamic-hrp2_10_plugins_dependencies dynamic)
SET(dynamic-hrp2_10_old_plugins_dependencies dynamic)

FOREACH(lib ${libs})
  ADD_LIBRARY(${lib} SHARED ${lib}.cpp)

  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${lib} "${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  IF (APPLE)
	TARGET_LINK_LIBRARIES(${lib} ${Boost_LIBRARIES})
  ENDIF (APPLE)

  PKG_CONFIG_USE_DEPENDENCY(${lib} jrl-dynamics)
  PKG_CONFIG_USE_DEPENDENCY(${lib} hrp2-dynamics)
  PKG_CONFIG_USE_DEPENDENCY(${lib} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${lib} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${lib} jrl-mal)

  INSTALL(TARGETS ${lib} DESTINATION lib/plugin)
  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${lib})

  DYNAMIC_GRAPH_PYTHON_MODULE("sot/dynamics/${PYTHON_LIBRARY_NAME}"
    ${lib}
    sot-dynamics-${PYTHON_LIBRARY_NAME}-wrap
    )
ENDFOREACH(lib)

# Install empty __init__.py files in intermediate directories.
CONFIG_FILES(dynamic_graph/sot/dynamics/hrp2.py)

INSTALL(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/dynamics/hrp2.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics/humanoid_robot.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics/parser.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics/tools.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics/solver.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dynamics
  )

PKG_CONFIG_USE_DEPENDENCY(dynamic-hrp2 hrp2-dynamics)

IF(${HRP2_10_OPTIMIZED_FOUND})
  PKG_CONFIG_USE_DEPENDENCY(dynamic-hrp2_10 hrp2-10-optimized)
  PKG_CONFIG_USE_DEPENDENCY(dynamic-hrp2_10_old hrp2-10-optimized)
ENDIF()
