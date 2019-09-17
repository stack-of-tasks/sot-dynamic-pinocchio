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
  ADD_UNIT_TEST(${test} ${test})

  TARGET_LINK_LIBRARIES(${test}
    dp-zmpreffromcom
    dp-force-compensation
    #integrator-force-exact
    dp-mass-apparent
    dp-integrator-force-rk4
    dp-integrator-force
    dp-angle-estimator
    dp-waist-attitude-from-sensor
    ${LIBRARY_NAME}
    )

  PKG_CONFIG_USE_DEPENDENCY(${test} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${test} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${test} dynamic-graph)

  IF(${test}_plugins_dependencies)
    ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  ENDIF(${test}_plugins_dependencies)

  TARGET_LINK_LIBRARIES(${test} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
ENDFOREACH(test)
