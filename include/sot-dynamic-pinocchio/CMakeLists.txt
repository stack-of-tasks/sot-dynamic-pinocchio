# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

SET(${PROJECT_NAME}_HEADERS
  dynamic-pinocchio.h
  integrator-force-exact.h
  zmpreffromcom.h
  integrator-force.h
  force-compensation.h
  mass-apparent.h
  waist-attitude-from-sensor.h
  matrix-inertia.h
  integrator-force-rk4.h
  angle-estimator.h
  )

INSTALL(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION include/${CUSTOM_HEADER_DIR})
