INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES kine_romeo.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

