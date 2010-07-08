# 
#  Copyright 
# 

SET(${PROJECT_NAME}_HEADERS
	dynamic.h
	integrator-force-exact.h
	zmpreffromcom.h
	integrator-force.h
	force-compensation.h
	mass-apparent.h
	waist-attitude-from-sensor.h
	matrix-inertia.h
	integrator-force-rk4.h
	angle-estimator.h
	dynamic-hrp2.h
)

# Recreate correct path for the headers
#--------------------------------------

SET(fullpath_${PROJECT_NAME}_HEADERS)
FOREACH(lHeader ${${PROJECT_NAME}_HEADERS})
  SET(fullpath_${PROJECT_NAME}_HEADERS 
    ${fullpath_${PROJECT_NAME}_HEADERS}
    ./${PROJECT_NAME}/${lHeader}
  )

ENDFOREACH(lHeader)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${fullpath_${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
