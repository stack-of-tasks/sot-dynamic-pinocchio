# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-dynamic-pinocchio)
SET(PROJECT_DESCRIPTION "pinocchio bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")
SET(PROJECT_SUFFIX "-v3")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
SET(CUSTOM_HEADER_DIR "sot/dynamic-pinocchio")
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/sphinx.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
ADD_PROJECT_DEPENDENCY(sot-tools REQUIRED PKG_CONFIG_REQUIRES sot-tools)
ADD_PROJECT_DEPENDENCY(example-robot-data)

SET(BOOST_COMPONENTS filesystem system unit_test_framework)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED
    PKG_CONFIG_REQUIRES dynamic-graph-python)
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()

# Main Library
SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/dynamic-pinocchio.h
  include/${CUSTOM_HEADER_DIR}/integrator-force-exact.h
  include/${CUSTOM_HEADER_DIR}/zmpreffromcom.h
  include/${CUSTOM_HEADER_DIR}/integrator-force.h
  include/${CUSTOM_HEADER_DIR}/force-compensation.h
  include/${CUSTOM_HEADER_DIR}/mass-apparent.h
  include/${CUSTOM_HEADER_DIR}/waist-attitude-from-sensor.h
  include/${CUSTOM_HEADER_DIR}/matrix-inertia.h
  include/${CUSTOM_HEADER_DIR}/integrator-force-rk4.h
  include/${CUSTOM_HEADER_DIR}/angle-estimator.h
  )

SET(${PROJECT_NAME}_SOURCES
  src/sot-dynamic-pinocchio
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES}
  sot-core::sot-core)

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
