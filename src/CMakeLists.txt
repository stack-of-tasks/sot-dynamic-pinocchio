# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)


# Main Library
ADD_LIBRARY(${PROJECT_NAME} SHARED sot-dynamic-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} dynamic-graph)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES})
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

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
  dynamic
  )

SET(integrator-force-rk4_plugins_dependencies integrator-force)
SET(integrator-force-exact_plugins_dependencies integrator-force)

FOREACH(lib ${plugins})
  SET(libname dp-${lib})
  ADD_LIBRARY(${libname} SHARED ${lib})
  SET_TARGET_PROPERTIES(${libname} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${libname} "dp-${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${libname} "dp-${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  TARGET_LINK_LIBRARIES(${libname} ${PROJECT_NAME})
  TARGET_LINK_LIBRARIES(${libname} ${Boost_LIBRARIES})

  PKG_CONFIG_USE_DEPENDENCY(${libname} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${libname} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${libname} dynamic-graph)

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${libname} DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  # build python submodule
  IF(BUILD_PYTHON_INTERFACE)
    PKG_CONFIG_USE_DEPENDENCY(${libname} eigenpy)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${lib})

    DYNAMIC_GRAPH_PYTHON_MODULE("sot/dynamics_pinocchio/${PYTHON_LIBRARY_NAME}"
      ${libname}
      sot-dynamic-pinocchio-${PYTHON_LIBRARY_NAME}-wrap
      1
      "src/python-module-py.cpp"
      )
    PKG_CONFIG_USE_DEPENDENCY(sot-dynamic-pinocchio-${PYTHON_LIBRARY_NAME}-wrap pinocchio)
    PKG_CONFIG_USE_DEPENDENCY(sot-dynamic-pinocchio-${PYTHON_LIBRARY_NAME}-wrap sot-core)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(lib)

# Install empty __init__.py files in intermediate directories.
IF(BUILD_PYTHON_INTERFACE)
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/humanoid_robot.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/tools.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dynamics_pinocchio/parser.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dynamics_pinocchio
    )
ENDIF(BUILD_PYTHON_INTERFACE)
