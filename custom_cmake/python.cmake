# Copyright (C) 2008-2016 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


#
# SOT_DYNAMIC_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME
# ---------------------------
#
# Add a python submodule to dynamic_graph
#
#  SUBMODULENAME : the name of the submodule (can be foo/bar),
#
#  LIBRARYNAME   : library to link the submodule with.
#
#  TARGETNAME    : name of the target: should be different for several
#                  calls to the macro.
#
#  NOTICE : Before calling this macro, set variable NEW_ENTITY_CLASS as
#           the list of new Entity types that you want to be bound.
#           Entity class name should match the name referencing the type
#           in the factory.
#
MACRO(SOT_DYNAMIC_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)
  FINDPYTHON()
  
  SET(PYTHON_MODULE ${TARGETNAME})
  
  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/src/python-module-py.cpp)
  #${PROJECT_SOURCE_DIR}/src/sot-dynamic-py.cpp)
  
  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME})
  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME dynamic_graph/${SUBMODULENAME}/wrap
    )
  
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${LIBRARYNAME} ${PYTHON_LIBRARY})
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${Boost_LIBRARIES})
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} pinocchio)
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} eigenpy)

  PKG_CONFIG_USE_DEPENDENCY(${PYTHON_MODULE} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${PYTHON_MODULE} pinocchio)
  PKG_CONFIG_USE_DEPENDENCY(${PYTHON_MODULE} eigenpy)
  
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  
  #
  # Installation
  #
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/dynamic_graph/${SUBMODULENAME})

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
    ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
    )

  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
    DESTINATION ${PYTHON_INSTALL_DIR}
    )

ENDMACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME)