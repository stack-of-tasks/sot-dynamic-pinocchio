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

    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} jrl-dynamics)
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} hrp2-dynamics)
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph)
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} jrl-mal)

    IF(${test}_plugins_dependencies)
        ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
        TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    ENDIF(${test}_plugins_dependencies)


	ADD_TEST(${test} ${EXECUTABLE_NAME})
ENDFOREACH(test)
