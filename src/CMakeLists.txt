# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)
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

INCLUDE(../custom_cmake/python.cmake)
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

FOREACH(lib ${plugins})
  SET(libname dp-${lib})
  ADD_LIBRARY(${libname} SHARED ${lib}.cpp)
  SET_TARGET_PROPERTIES(${libname} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${libname} "dp-${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${libname} "dp-${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  TARGET_LINK_LIBRARIES(${libname} ${Boost_LIBRARIES})

  PKG_CONFIG_USE_DEPENDENCY(${libname} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${libname} eigenpy)
  PKG_CONFIG_USE_DEPENDENCY(${libname} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${libname} dynamic-graph)

  INSTALL(TARGETS ${libname} DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${lib})

  SOT_DYNAMIC_PYTHON_MODULE("sot/dynamics_pinocchio/${PYTHON_LIBRARY_NAME}"
    ${libname}
    sot-dynamic-pinocchio-${PYTHON_LIBRARY_NAME}-wrap
    )
  UNSET({libname})
ENDFOREACH(lib)

# Main Library
ADD_LIBRARY(${LIBRARY_NAME} SHARED sot-dynamic-pinocchio.cpp)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pinocchio)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-core)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

TARGET_LINK_LIBRARIES(dp-dynamic ${LIBRARY_NAME})

# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/humanoid_robot.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/tools.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/parser.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dynamics_pinocchio
  )