// Copyright (C) 2008-2016, 2019 LAAS-CNRS, JRL AIST-CNRS.

#define PY_SSIZE_T_CLEAN

#include <sot/core/debug.hh>

#include <sot-dynamic-pinocchio/dynamic-pinocchio.h>
#include <Python.h>
#include <boost/python.hpp>
#include <typeinfo>
#include <cstdio>

namespace dynamicgraph {
namespace sot {

PyObject* setPinocchioModel(PyObject* /* self */, PyObject* args) {
  PyObject* object = NULL;
  PyObject* pyPinocchioObject;
  void* pointer1 = NULL;
  pinocchio::Model* pointer2 = NULL;
  if (!PyArg_ParseTuple(args, "OO", &object, &pyPinocchioObject)) return NULL;

  if (!PyCapsule_CheckExact(object)) {
    PyErr_SetString(PyExc_TypeError, "function takes a PyCapsule as argument");
    return NULL;
  }

  pointer1 = PyCapsule_GetPointer(object, "dynamic_graph.Entity");
  DynamicPinocchio* dyn_entity = (DynamicPinocchio*)pointer1;

  try {
    boost::python::extract<pinocchio::Model&> cppHandle(pyPinocchioObject);
    pointer2 = (pinocchio::Model*)&cppHandle();
    dyn_entity->setModel(pointer2);
  } catch (const std::exception& exc) {
    // PyErr_SetString(dgpyError, exc.what());
    return NULL;
  } catch (const char* s) {
    // PyErr_SetString(dgpyError, s);
    return NULL;
  } catch (...) {
    // PyErr_SetString(dgpyError, "Unknown exception");
    return NULL;
  }

  return Py_BuildValue("");
}

PyObject* setPinocchioData(PyObject* /* self */, PyObject* args) {
  PyObject* object = NULL;
  PyObject* pyPinocchioObject;
  void* pointer1 = NULL;
  pinocchio::Data* pointer2 = NULL;
  if (!PyArg_ParseTuple(args, "OO", &object, &pyPinocchioObject)) return NULL;

  if (!PyCapsule_CheckExact(object)) {
    PyErr_SetString(PyExc_TypeError, "function takes a PyCapsule as argument");
    return NULL;
  }

  pointer1 = PyCapsule_GetPointer(object, "dynamic_graph.Entity");
  DynamicPinocchio* dyn_entity = (DynamicPinocchio*)pointer1;

  try {
    boost::python::extract<pinocchio::Data&> cppHandle(pyPinocchioObject);
    pointer2 = (pinocchio::Data*)&cppHandle();
    dyn_entity->setData(pointer2);
  } catch (const std::exception& exc) {
    //	PyErr_SetString(dgpyError, exc.what());
    return NULL;
  } catch (const char* s) {
    //	PyErr_SetString(dgpyError, s);
    return NULL;
  } catch (...) {
    //	PyErr_SetString(dgpyError, "Unknown exception");
    return NULL;
  }

  return Py_BuildValue("");
}
}  // namespace sot
}  // namespace dynamicgraph
/**
   \brief List of python functions
*/

static PyMethodDef functions[] = {
    /*  {"get_pinocchio_model",  dynamicgraph::sot::getPinocchioModel, METH_VARARGS,
        "Get the pinocchio model as python object"},*/
    {"set_pinocchio_model", dynamicgraph::sot::setPinocchioModel, METH_VARARGS,
     "Set the model from pinocchio python object"},
    {"set_pinocchio_data", dynamicgraph::sot::setPinocchioData, METH_VARARGS,
     "Set the data from pinocchio python object"},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef SotDynamicPinocchioModuleDef = {
    PyModuleDef_HEAD_INIT,
    "wrap",
    NULL,
    0,
    functions,
    NULL,
    NULL,
    NULL,
    NULL};
#define INITERROR return NULL
#else
#define INITERROR return
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC PyInit_wrap(void)
#else
void initwrap(void)
#endif
{
#if PY_MAJOR_VERSION >= 3
  PyObject* module = PyModule_Create(&SotDynamicPinocchioModuleDef);
#else
  PyObject* module = Py_InitModule("wrap", functions);
#endif

  if (module == NULL) INITERROR;

#if PY_MAJOR_VERSION >= 3
  return module;
#endif
}

#ifdef __cplusplus
}  // extern "C"
#endif

