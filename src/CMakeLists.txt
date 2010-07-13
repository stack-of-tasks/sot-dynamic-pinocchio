# 
#  Copyright 
# 

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

IF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	ADD_DEFINITIONS(-DDEBUG=2)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

# provide path to library libdynamicsJRLJapan.so
LINK_DIRECTORIES(${DYNAMICSJRLJAPAN_LIBRARY_DIRS})


# provide path to library libhrp2Dynamics.so
LINK_DIRECTORIES(${HRP2DYNAMICS_LIBRARY_DIRS})


# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})


# provide path to library libsot-core.so
LINK_DIRECTORIES(${SOT_CORE_LIBRARY_DIRS})


# provide path to library libMatrixAbstractLayer.so
LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})

# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})

IF(WIN32)
foreach(dlink ${DYNAMICSJRLJAPAN_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})

IF(WIN32)
foreach(dlink ${HRP2DYNAMICS_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})

IF(WIN32)
foreach(dlink ${DYNAMIC_GRAPH_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add sot-core compilation flags and link to library libsot-core.so
ADD_DEFINITIONS(${SOT_CORE_CFLAGS})

IF(WIN32)
foreach(dlink ${SOT_CORE_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})

IF(WIN32)
foreach(dlink ${MATRIXABSTRACTLAYER_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

SET(libs
	zmpreffromcom
	force-compensation
	integrator-force-exact
	mass-apparent
	integrator-force-rk4
	integrator-force
	angle-estimator
	waist-attitude-from-sensor
)

IF(${DYNAMICSJRLJAPAN_FOUND})
  LIST(APPEND libs dynamic)
	IF(${_hrp2Dynamics_FOUND})
	  LIST(APPEND libs dynamic-hrp2)
	  SET (dynamic-hrp2_plugins_dependencies dynamic)
	ENDIF(${_hrp2Dynamics_FOUND})
ENDIF(${DYNAMICSJRLJAPAN_FOUND})

SET(integrator-force-rk4_plugins_dependencies integrator-force)
SET(integrator-force-exact_plugins_dependencies integrator-force)

FOREACH(lib ${libs})
	ADD_LIBRARY(${lib}
	  SHARED
	  ${lib}.cpp)
	
	SET_TARGET_PROPERTIES(${lib} 
	  PROPERTIES 
	  SOVERSION ${PROJECT_VERSION}
	  PREFIX ""
	  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/plugin)
	  
	IF (UNIX)  
		TARGET_LINK_LIBRARIES(${lib} ${DYNAMIC_GRAPH_LIBRARIES})
		TARGET_LINK_LIBRARIES(${lib} ${SOT_CORE_LIBRARIES})
		TARGET_LINK_LIBRARIES(${lib} ${MATRIXABSTRACTLAYER_LIBRARIES})
		TARGET_LINK_LIBRARIES(${lib} ${HRP2DYNAMICS_LIBRARIES})
		TARGET_LINK_LIBRARIES(${lib} ${DYNAMICSJRLJAPAN_LIBRARIES})
	ENDIF(UNIX)
	
	IF(WIN32)
		SET_TARGET_PROPERTIES(${lib}
		  PROPERTIES
		  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
		)
	ENDIF(WIN32)
	
	IF(${lib}_plugins_dependencies)
		ADD_DEPENDENCIES(${lib} "${${lib}_plugins_dependencies}")
		TARGET_LINK_LIBRARIES(${lib} "${${lib}_plugins_dependencies}")
	ENDIF(${lib}_plugins_dependencies)
	
	INSTALL(TARGETS ${lib}
  	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/plugin)
ENDFOREACH(lib)

  
