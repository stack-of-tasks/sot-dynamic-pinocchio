/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/factory.h>
#include <sot/dynamic-pinocchio/dynamic-pinocchio.h>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

extern "C" {
::dynamicgraph::Entity* EntityMaker_DynamicPinocchio(
    const std::string& objname) {
  return new DynamicPinocchio(objname);
}
::dynamicgraph::EntityRegisterer reg_Dynamic("DynamicPinocchio",
                                             &EntityMaker_DynamicPinocchio);
}
// DYNAMICGRAPH_FACTORY_DYNAMIC_PLUGIN(Dynamic,"Dynamic");
