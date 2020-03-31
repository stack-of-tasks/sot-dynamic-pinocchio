# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

SET(tests
  test_constructor
  #test_config
  #  dummy
  #  test_djj
  #  test_dyn
  #  test_results
  )

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
  ADD_UNIT_TEST(${test} ${test}.cpp)

  TARGET_LINK_LIBRARIES(${test}
    ${PROJECT_NAME} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${${test}_deps}
    zmpreffromcom
    force-compensation
    #integrator-force-exact
    mass-apparent
    integrator-force-rk4
    integrator-force
    angle-estimator
    waist-attitude-from-sensor
    )
ENDFOREACH(test)

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)

