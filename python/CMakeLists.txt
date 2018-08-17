IF(BUILD_PYTHON_INTERFACE)
  INSTALL(
    FILES kine_romeo.py kine_romeo_small.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
    )
ENDIF(BUILD_PYTHON_INTERFACE)
