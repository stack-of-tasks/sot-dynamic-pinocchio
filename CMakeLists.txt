# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-dynamic-pinocchio)
SET(PROJECT_DESCRIPTION "pinocchio bindings for dynamic-graph.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")
SET(PROJECT_SUFFIX "-v3")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)


# Specify the project.
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

find_package (Boost REQUIRED
  python filesystem system thread program_options unit_test_framework)
find_package (Eigen3 REQUIRED NO_MODULE)

INCLUDE(cmake/python.cmake)
INCLUDE(cmake/test.cmake)

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(DOXYGEN_USE_MATHJAX YES)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )


# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.0.0")
ADD_PROJECT_DEPENDENCY(dynamic-graph REQUIRED )
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED )
#ADD_PROJECT_DEPENDENCY(sot-tools REQUIRED )

SET(BOOST_COMPONENTS filesystem system unit_test_framework)

OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION (INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION
  "Suffix shared library name by a string depending on git status of project"
  ON)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED)
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_PATH})
  ADD_REQUIRED_DEPENDENCY("eigenpy")
ENDIF(BUILD_PYTHON_INTERFACE)

# Add dependency toward sot-dynamic-pinocchio library in pkg-config file.
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

#SEARCH_FOR_BOOST()
#SEARCH_FOR_EIGEN()

# Add subdirectories.
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  ADD_SUBDIRECTORY(include/${CUSTOM_HEADER_DIR})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unitTesting)
