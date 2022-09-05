# Copyright 2010-2021, JRL, CNRS/AIST, LAAS-CNRS François Bleibel Olivier Stasse
# Florent Lamiraux Guilhem Saurel

# ROS buildfarm uses isolated environments, in which the build rpath is used
set(BUILD_WITH_INSTALL_RPATH TRUE)

set(plugins
    zmpreffromcom
    force-compensation
    integrator-force-exact
    mass-apparent
    integrator-force-rk4
    integrator-force
    angle-estimator
    waist-attitude-from-sensor
    zmp-from-forces
    dynamic)

set(integrator-force-rk4_plugins_deps integrator-force)
set(integrator-force-exact_plugins_deps integrator-force lapack)
set(mass-apparent_plugins_deps integrator-force)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED ${plugin})
  set_target_properties(${LIBRARY_NAME} PROPERTIES INSTALL_RPATH $ORIGIN)

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME}
                                               ${${LIBRARY_NAME}_plugins_deps})

  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  if(BUILD_PYTHON_INTERFACE)
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      dynamic_graph_python_module(
        "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      dynamic_graph_python_module(
        "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  endif(BUILD_PYTHON_INTERFACE)
endforeach(plugin)

if(BUILD_PYTHON_INTERFACE)
  set(${PY_NAME}_PYTHON __init__.py humanoid_robot.py tools.py kine_romeo.py
                        kine_romeo_small.py)

  foreach(py_file ${${PY_NAME}_PYTHON})
    python_install_on_site("dynamic_graph/${PYTHON_DIR}" ${py_file})
  endforeach(py_file ${${PY_NAME}_PYTHON})
endif(BUILD_PYTHON_INTERFACE)
