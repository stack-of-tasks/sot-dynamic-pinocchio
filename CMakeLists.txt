# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-dynamic-pinocchio)
SET(PROJECT_DESCRIPTION "pinocchio bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")
SET(PROJECT_SUFFIX "-v3")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(DOXYGEN_USE_MATHJAX YES)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()
# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-tools >= 2.0.0")

SET(BOOST_COMPONENTS filesystem system unit_test_framework)

OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  ADD_REQUIRED_DEPENDENCY("eigenpy")
ENDIF(BUILD_PYTHON_INTERFACE)


# List plug-ins that will be compiled.
SET(plugins
  zmpreffromcom
  force-compensation
  integrator-force-exact
  mass-apparent
  integrator-force-rk4
  integrator-force
  angle-estimator
  waist-attitude-from-sensor
  zmp-from-forces
  )

SET(LIBRARY_NAME ${PROJECT_NAME})
LIST(APPEND plugins dynamic)

# Add dependency toward sot-dynamic-pinocchio library in pkg-config file.
PKG_CONFIG_APPEND_LIBS(${LIBRARY_NAME})

SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
