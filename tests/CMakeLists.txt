# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

set(tests test_constructor
          # test_config dummy test_djj test_dyn test_results
)

set(test_dyn_plugins_dependencies dynamic)

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# getting the information for the robot. SET(samplemodelpath
# ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/) SET(samplespec
# ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleSpecificities.xml
# ) SET(sampleljr
# ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleLinkJointRank.xml
# ) SET(sampleinitconfig
# ${JRL_DYNAMICS_PKGDATAROOTDIR}/jrl-dynamics/examples/data/sampleInitConfig.dat)

list(APPEND LOGGING_WATCHED_VARIABLES samplespec sampleljr)

foreach(test ${tests})
  add_unit_test(${test} ${test}.cpp)

  target_link_libraries(
    ${test}
    ${PROJECT_NAME}
    Boost::unit_test_framework
    ${${test}_deps}
    zmpreffromcom
    force-compensation
    # integrator-force-exact
    mass-apparent
    integrator-force-rk4
    integrator-force
    angle-estimator
    waist-attitude-from-sensor)
endforeach(test)

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
