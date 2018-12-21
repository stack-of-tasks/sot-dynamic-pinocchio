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

ADD_DEFINITIONS(-DDEBUG=2)

SET(tests
  test_constructor
  test_pinocchio_device
#test_config
#  dummy
#  test_djj
#  test_dyn
#  test_results
)

SET(test_pinocchio_device_PKG_CONFIG
  yaml-cpp
  )


#----------------------------------------------------
# Install procedure for the urdf files
#----------------------------------------------------

SET(test_dyn_plugins_dependencies dynamic)


# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)


# getting the information for the robot.
#SET(samplemodelpath ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/)
#SET(samplespec
#  ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleSpecificities.xml
#  )
#SET(sampleljr
#  ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleLinkJointRank.xml
#  )
#SET(sampleinitconfig
#  ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleInitConfig.dat)

LIST(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

FOREACH(test ${tests})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    dp-zmpreffromcom
    dp-force-compensation
    #integrator-force-exact
    dp-mass-apparent
    dp-integrator-force-rk4
    dp-integrator-force
    dp-angle-estimator
    dp-waist-attitude-from-sensor
    dp-pinocchio-device
    ${LIBRARY_NAME}
    )

  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph)

  # Check if the test needs to use python interface.
  IF(BUILD_PYTHON_INTERFACE)
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph-python)
  ENDIF(BUILD_PYTHON_INTERFACE)

  # Uses PKG config additional option
  MESSAGE("DISPLAY ${test}_PKG_CONFIG: ${${test}_PKG_CONFIG}")
  IF( ${test}_PKG_CONFIG )
    PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} ${${test}_PKG_CONFIG})    
  ENDIF( ${test}_PKG_CONFIG )

  IF(${test}_plugins_dependencies)
    ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  ENDIF(${test}_plugins_dependencies)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  ADD_TEST(${test} ${EXECUTABLE_NAME}
    ${samplemodelpath} ${samplespec} ${sampleljr} )

  IF (UNIX)
    SET(EXTRA_LD_LIBRARY_PATH $ENV{LD_LIBRARY_PATH})
    SET_PROPERTY(TEST ${test} PROPERTY
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:${CMAKE_BINARY_DIR}/src:${BOOST_ROOT}/lib:${EXTRA_LD_LIBRARY_PATH}")
  ENDIF(UNIX)

ENDFOREACH(test)
