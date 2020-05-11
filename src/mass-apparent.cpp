/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/dynamic-pinocchio/mass-apparent.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MassApparent, "MassApparent");

MassApparent::MassApparent(const std::string& name)
    : Entity(name),
      jacobianSIN(NULL, "sotMassApparent(" + name + ")::input(vector)::jacobian"),
      inertiaInverseSIN(NULL, "sotMassApparent(" + name + ")::input(vector)::inertiaInverse"),
      massInverseSOUT(boost::bind(&MassApparent::computeMassInverse, this, _1, _2), jacobianSIN << inertiaInverseSIN,
                      "sotMassApparent(" + name + ")::output(Vector)::massInverse"),
      massSOUT(boost::bind(&MassApparent::computeMass, this, _1, _2), massInverseSOUT,
               "sotMassApparent(" + name + ")::output(Vector)::mass")

      ,
      inertiaSIN(NULL, "sotMassApparent(" + name + ")::input(vector)::inertia"),
      inertiaInverseSOUT(boost::bind(&MassApparent::computeInertiaInverse, this, _1, _2), inertiaSIN,
                         "sotMassApparent(" + name + ")::input(vector)::inertiaInverseOUT") {
  sotDEBUGIN(5);

  signalRegistration(jacobianSIN);
  signalRegistration(inertiaInverseSIN);
  signalRegistration(massInverseSOUT);
  signalRegistration(massSOUT);
  signalRegistration(inertiaSIN);
  signalRegistration(inertiaInverseSOUT);
  inertiaInverseSIN.plug(&inertiaInverseSOUT);

  sotDEBUGOUT(5);
}

MassApparent::~MassApparent(void) { return; }

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
dynamicgraph::Matrix& MassApparent::computeMassInverse(dynamicgraph::Matrix& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix& J = jacobianSIN(time);
  const dynamicgraph::Matrix& A = inertiaInverseSIN(time);

  dynamicgraph::Matrix AJt(J.cols(), J.rows());
  AJt = A * J.transpose();

  res.resize(J.rows(), J.rows());
  res = J * AJt;

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Matrix& MassApparent::computeMass(dynamicgraph::Matrix& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix& omega = massInverseSOUT(time);
  res = omega.inverse();

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Matrix& MassApparent::computeInertiaInverse(dynamicgraph::Matrix& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix& A = inertiaSIN(time);
  res = A.inverse();

  sotDEBUGOUT(15);
  return res;
}
