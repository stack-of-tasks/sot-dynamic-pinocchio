// Copyright (C) 2008-2016 LAAS-CNRS, JRL AIST-CNRS.

#include <sot/core/debug.hh>

#include <sot-dynamic-pinocchio/dynamic-pinocchio.h>
#include <Python.h>
#include <boost/python.hpp>
#include <typeinfo>
#include <cstdio>


namespace dynamicgraph{
  namespace sot{

    PyObject* setPinocchioModel(PyObject* /* self */,PyObject* args) {
      PyObject* object = NULL;
      PyObject* pyPinocchioObject;
      void* pointer1 = NULL;
      pinocchio::Model* pointer2 = NULL;
      if (!PyArg_ParseTuple(args, "OO", &object, &pyPinocchioObject))
	return NULL;

      if (!PyCObject_Check(object)) {
	PyErr_SetString(PyExc_TypeError,
			"function takes a PyCObject as argument");
	return NULL;
      }

      pointer1 = PyCObject_AsVoidPtr(object);
      DynamicPinocchio* dyn_entity = (DynamicPinocchio*) pointer1;

      try {
	boost::python::extract<pinocchio::Model&> cppHandle(pyPinocchioObject);
	pointer2 = (pinocchio::Model*) &cppHandle();
	dyn_entity->setModel(pointer2);
      }
      catch (const std::exception& exc) {
	//PyErr_SetString(dgpyError, exc.what());
	return NULL;
      }
      catch (const char* s) {
	//PyErr_SetString(dgpyError, s);
	return NULL;
      }
      catch (...) {
	//PyErr_SetString(dgpyError, "Unknown exception");
	return NULL;
      }

      return Py_BuildValue("");
    }

    PyObject* setPinocchioData(PyObject* /* self */,PyObject* args) {
      PyObject* object = NULL;
      PyObject* pyPinocchioObject;
      void* pointer1 = NULL;
      pinocchio::Data* pointer2 = NULL;
      if (!PyArg_ParseTuple(args, "OO", &object, &pyPinocchioObject))
	return NULL;

      if (!PyCObject_Check(object)) {
	PyErr_SetString(PyExc_TypeError,
			"function takes a PyCObject as argument");
	return NULL;
      }

      pointer1 = PyCObject_AsVoidPtr(object);
      DynamicPinocchio* dyn_entity = (DynamicPinocchio*) pointer1;

      try {
	boost::python::extract<pinocchio::Data&> cppHandle(pyPinocchioObject);
	pointer2 = (pinocchio::Data*) &cppHandle();
	dyn_entity->setData(pointer2);
      }
      catch (const std::exception& exc) {
	//	PyErr_SetString(dgpyError, exc.what());
	return NULL;
      }
      catch (const char* s) {
	//	PyErr_SetString(dgpyError, s);
	return NULL;
      }
      catch (...) {
	//	PyErr_SetString(dgpyError, "Unknown exception");
	return NULL;
      }

      return Py_BuildValue("");
    }
  }
}
/**
   \brief List of python functions
*/

static PyMethodDef functions[] = {
  /*  {"get_pinocchio_model",  dynamicgraph::sot::getPinocchioModel, METH_VARARGS,
      "Get the pinocchio model as python object"},*/
  {"set_pinocchio_model",  dynamicgraph::sot::setPinocchioModel, METH_VARARGS,
   "Set the model from pinocchio python object"},
  {"set_pinocchio_data",  dynamicgraph::sot::setPinocchioData, METH_VARARGS,
   "Set the data from pinocchio python object"},
  {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initwrap(void)
{
    PyObject *m;

    m = Py_InitModule("wrap", functions);
    if (m == NULL)
        return;
}
