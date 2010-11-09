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

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

# provide path to library libsot-core.so
LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)

# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
ADD_DEFINITIONS(${JRL_MAL_CFLAGS})

IF(WIN32)
foreach(dlink ${JRL_MAL_LDFLAGS} 
			  ${DYNAMIC_GRAPH_LDFLAGS}  ${SOT_CORE_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

SET(tests
	dummy
	test_djj
    test_dyn
	test_results)

SET(test_dyn_plugins_dependencies dynamic)

FOREACH(test ${tests})
	SET(EXECUTABLE_NAME "${test}_exe")
	ADD_EXECUTABLE(${EXECUTABLE_NAME}
	  ${test}.cpp)

	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
	  	zmpreffromcom
		force-compensation
		integrator-force-exact
		mass-apparent
		integrator-force-rk4
		integrator-force
		angle-estimator
		waist-attitude-from-sensor
		)

		IF (UNIX)
			TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
			TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${SOT_CORE_LIBRARIES})
			TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${JRL_MAL_LIBRARIES})
		ENDIF(UNIX)

        PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} jrl-dynamics)
        PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} hrp2-dynamics)

		IF(WIN32)
		SET_TARGET_PROPERTIES(${EXECUTABLE_NAME}
		 PROPERTIES
		  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
		)
	 	ENDIF(WIN32)

    IF(${test}_plugins_dependencies)
        ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
        TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    ENDIF(${test}_plugins_dependencies)


	ADD_TEST(${test} ${EXECUTABLE_NAME})
ENDFOREACH(test)
