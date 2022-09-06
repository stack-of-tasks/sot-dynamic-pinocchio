# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-dynamic-pinocchio)
set(PROJECT_DESCRIPTION "pinocchio bindings for dynamic-graph.")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "sot/dynamic-pinocchio")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/lapack.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(sot-tools REQUIRED)

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_project_dependency(example-robot-data 3.8.0 REQUIRED)
endif(BUILD_TESTING)

if(Boost_VERSION GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost python at
  # least fo boost 1.73 to 1.75
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/dynamic-pinocchio.h
    include/${CUSTOM_HEADER_DIR}/integrator-force-exact.h
    include/${CUSTOM_HEADER_DIR}/zmpreffromcom.h
    include/${CUSTOM_HEADER_DIR}/integrator-force.h
    include/${CUSTOM_HEADER_DIR}/force-compensation.h
    include/${CUSTOM_HEADER_DIR}/mass-apparent.h
    include/${CUSTOM_HEADER_DIR}/waist-attitude-from-sensor.h
    include/${CUSTOM_HEADER_DIR}/matrix-inertia.h
    include/${CUSTOM_HEADER_DIR}/integrator-force-rk4.h
    include/${CUSTOM_HEADER_DIR}/angle-estimator.h)

set(${PROJECT_NAME}_SOURCES src/sot-dynamic-pinocchio)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} sot-core::sot-core)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)
