INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES kine_romeo.py kine_romeo_small.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

