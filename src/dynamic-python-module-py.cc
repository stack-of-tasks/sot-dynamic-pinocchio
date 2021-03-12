#include <dynamic-graph/python/module.hh>

#include <sot/dynamic-pinocchio/dynamic-pinocchio.h>

namespace dg = dynamicgraph;
namespace dgs = dynamicgraph::sot;

typedef bp::return_value_policy<bp::reference_existing_object> reference_existing_object;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");
  bp::import("pinocchio");

  dg::python::exposeEntity<dgs::DynamicPinocchio, bp::bases<dg::Entity>, dg::python::AddCommands>()
      .add_property("model", bp::make_function(&dgs::DynamicPinocchio::getModel, reference_existing_object()),
                    bp::make_function(&dgs::DynamicPinocchio::setModel))
      .add_property("data", bp::make_function(&dgs::DynamicPinocchio::getData, reference_existing_object()),
                    bp::make_function(&dgs::DynamicPinocchio::setData))
      .def("setModel", &dgs::DynamicPinocchio::setModel)
      .def("createData", &dgs::DynamicPinocchio::createData)
      .def("setData", &dgs::DynamicPinocchio::setData);
}
