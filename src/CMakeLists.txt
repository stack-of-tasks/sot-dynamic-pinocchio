# Copyright 2010-2021, JRL, CNRS/AIST, LAAS-CNRS
# François Bleibel
# Olivier Stasse
# Florent Lamiraux
# Guilhem Saurel


# ROS buildfarm uses isolated environments, in which the build rpath is used
SET(BUILD_WITH_INSTALL_RPATH TRUE)

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
  dynamic
  )

SET(integrator-force-rk4_plugins_deps integrator-force)
SET(integrator-force-exact_plugins_deps integrator-force lapack)
SET(mass-apparent_plugins_deps integrator-force)

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin})
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES INSTALL_RPATH $ORIGIN)

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
    PUBLIC ${PROJECT_NAME} ${${LIBRARY_NAME}_plugins_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        MODULE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

IF(BUILD_PYTHON_INTERFACE)
  SET(${PY_NAME}_PYTHON
    __init__.py
    humanoid_robot.py
    tools.py
    kine_romeo.py
    kine_romeo_small.py
    )

  FOREACH(py_file ${${PY_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${py_file})
  ENDFOREACH(py_file ${${PY_NAME}_PYTHON})
ENDIF(BUILD_PYTHON_INTERFACE)
