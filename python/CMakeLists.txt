SET(${PROJECT_NAME}_PYTHON
  __init__.py
  kine_romeo.py
  kine_romeo_small.py
  )

IF(BUILD_PYTHON_INTERFACE)
  FOREACH(file ${${PROJECT_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
  ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})
ENDIF(BUILD_PYTHON_INTERFACE)
