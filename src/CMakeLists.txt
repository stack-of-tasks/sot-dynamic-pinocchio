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

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

IF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

# provide path to libraries
LINK_DIRECTORIES(${DYNAMICSJRLJAPAN_LIBRARY_DIRS})
LINK_DIRECTORIES(${HRP2_DYNAMICS_LIBRARY_DIRS})
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})
LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})
LINK_DIRECTORIES(${JRL_MAL_LIBRARY_DIRS})

# Add compilation flags
ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})
ADD_DEFINITIONS(${HRP2_DYNAMICS_CFLAGS})
ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
ADD_DEFINITIONS(${SOT_CORE_CFLAGS})
ADD_DEFINITIONS(${JRL_MAL_CFLAGS})

foreach(dlink 	${DYNAMICSJRLJAPAN_LDFLAGS}
    ${HRP2_DYNAMICS_LDFLAGS}
    ${DYNAMIC_GRAPH_LDFLAGS}
    ${SOT_CORE_LDFLAGS}
    ${JRL_MAL_LDFLAGS}
    )
  SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)

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

IF(${DYNAMICSJRLJAPAN_FOUND})
  LIST(APPEND libs dynamic)
  IF(${HRP2_DYNAMICS_FOUND})
    LIST(APPEND libs dynamic-hrp2)
    SET (dynamic-hrp2_plugins_dependencies dynamic)
  ENDIF(${HRP2_DYNAMICS_FOUND})
ENDIF(${DYNAMICSJRLJAPAN_FOUND})

IF(${_hrp210optimized_FOUND})
  LIST(APPEND libs dynamic-hrp2_10)
  SET (dynamic-hrp2_10_additional_libs ${_hrp210optimized_LIBRARIES})
  SET (dynamic-hrp2_10_plugins_dependencies dynamic)
  SET(dynamic-hrp2_10_plugins_compile_flags
    ${_dynamicsJRLJapan_CFLAGS} ${_hrp210optimized_CFLAGS})

  LIST(APPEND libs dynamic-hrp2_10_old)
  SET (dynamic-hrp2_10_old_additional_libs ${_hrp210optimized_LIBRARIES})
  SET (dynamic-hrp2_10_old_plugins_dependencies dynamic)
  SET(dynamic-hrp2_10_old_plugins_compile_flags
    ${_dynamicsJRLJapan_CFLAGS} ${_hrp210optimized_CFLAGS})

ENDIF(${_hrp210optimized_FOUND})

IF(NOT UNIX)
  SET(dynamic-hrp2_10_plugins_link_flags ${_dynamicsJRLJapan_LDFLAGS} ${_hrp210optimized_LDFLAGS})
  SET(dynamic-hrp2_10_old_plugins_link_flags ${_dynamicsJRLJapan_LDFLAGS} ${_hrp210optimized_LDFLAGS})
ENDIF(NOT UNIX)

SET(integrator-force-rk4_plugins_dependencies integrator-force)
SET(integrator-force-exact_plugins_dependencies integrator-force)

FOREACH(lib ${libs})
  ADD_LIBRARY(${lib}
    SHARED
    ${lib}.cpp)

  SET_TARGET_PROPERTIES(${lib}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    PREFIX ""
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/plugin)

  IF (UNIX)
    TARGET_LINK_LIBRARIES(${lib} ${DYNAMIC_GRAPH_LIBRARIES})
    #MESSAGE(FATAL_ERROR X ${DYNAMIC_GRAPH_LIBRARIES})
    TARGET_LINK_LIBRARIES(${lib} ${SOT_CORE_LIBRARIES})
    TARGET_LINK_LIBRARIES(${lib} ${JRL_MAL_LIBRARIES})
    TARGET_LINK_LIBRARIES(${lib} ${HRP2_DYNAMICS_LIBRARIES})
    TARGET_LINK_LIBRARIES(${lib} ${DYNAMICSJRLJAPAN_LIBRARIES})
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_additional_libs}")
  ENDIF(UNIX)

  IF(WIN32)
    SET(${lib}_plugins_link_flags "${${PROJECT_NAME}_src_LDFLAGS} ${${lib}_plugins_link_flags}")
  ENDIF(WIN32)

  SET_TARGET_PROPERTIES(${lib}
    PROPERTIES
    COMPILE_FLAGS "${${lib}_plugins_compile_flags}"
    LINK_FLAGS "${${lib}_plugins_link_flags}")

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${lib} "${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  INSTALL(TARGETS ${lib}
    DESTINATION lib/plugin)
ENDFOREACH(lib)
