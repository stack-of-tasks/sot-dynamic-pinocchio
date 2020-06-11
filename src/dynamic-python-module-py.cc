#include <dynamic-graph/python/module.hh>

#include <sot/dynamic-pinocchio/dynamic-pinocchio.h>

namespace dg = dynamicgraph;
namespace dgs = dynamicgraph::sot;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamicgraph");
  bp::import("pinocchio");

  dg::python::exposeEntity<dgs::DynamicPinocchio>()
    .def_readwrite("model", &dgs::DynamicPinocchio::m_model);
}
