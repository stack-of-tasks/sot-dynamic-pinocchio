set(${PROJECT_NAME}_PYTHON_TESTS humanoid_robot)
foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  add_python_unit_test("py-${TEST}" "tests/python/${TEST}.py")
endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
