/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/factory.h>
#include <sot/dynamic-pinocchio/integrator-force.h>

#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(IntegratorForce, "IntegratorForce");

const double IntegratorForce::TIME_STEP_DEFAULT = 5e-3;

IntegratorForce::IntegratorForce(const std::string& name)
    : Entity(name),
      timeStep(TIME_STEP_DEFAULT),
      forceSIN(NULL, "sotIntegratorForce(" + name + ")::input(vector)::force"),
      massInverseSIN(
          NULL, "sotIntegratorForce(" + name + ")::input(matrix)::massInverse"),
      frictionSIN(NULL,
                  "sotIntegratorForce(" + name + ")::input(matrix)::friction"),
      velocityPrecSIN(NULL,
                      "sotIntegratorForce(" + name + ")::input(matrix)::vprec")

      ,
      velocityDerivativeSOUT(
          boost::bind(&IntegratorForce::computeDerivative, this, _1, _2),
          velocityPrecSIN << forceSIN << massInverseSIN << frictionSIN,
          "sotIntegratorForce(" + name +
              ")::output(Vector)::velocityDerivative"),
      velocitySOUT(boost::bind(&IntegratorForce::computeIntegral, this, _1, _2),
                   velocityPrecSIN << velocityDerivativeSOUT,
                   "sotIntegratorForce(" + name + ")::output(Vector)::velocity")

      ,
      massSIN(NULL, "sotIntegratorForce(" + name + ")::input(matrix)::mass"),
      massInverseSOUT(
          boost::bind(&IntegratorForce::computeMassInverse, this, _1, _2),
          massSIN,
          "sotIntegratorForce(" + name + ")::input(matrix)::massInverseOUT") {
  sotDEBUGIN(5);

  signalRegistration(forceSIN);
  signalRegistration(massInverseSIN);
  signalRegistration(frictionSIN);
  signalRegistration(velocityPrecSIN);
  signalRegistration(velocityDerivativeSOUT);
  signalRegistration(velocitySOUT);
  signalRegistration(massInverseSOUT);
  signalRegistration(massSIN);

  massInverseSIN.plug(&massInverseSOUT);

  sotDEBUGOUT(5);
}

IntegratorForce::~IntegratorForce(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

/* The derivative of the signal is such that: M v_dot + B v = f. We deduce:
 * v_dot =  M^-1 (f - Bv)
 */
dynamicgraph::Vector& IntegratorForce::computeDerivative(
    dynamicgraph::Vector& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector& force = forceSIN(time);
  const dynamicgraph::Matrix& massInverse = massInverseSIN(time);
  const dynamicgraph::Matrix& friction = frictionSIN(time);

  sotDEBUG(15) << "force = " << force << std::endl;

  dynamicgraph::Vector f_bv(force.size());

  if (velocityPrecSIN) {
    const dynamicgraph::Vector& vel = velocityPrecSIN(time);
    sotDEBUG(15) << "vel = " << vel << std::endl;
    f_bv = friction * vel;
    f_bv *= -1;
  } else {
    f_bv.fill(0);
  }  // vel is not set yet.

  f_bv += force;
  res = massInverse * f_bv;

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Vector& IntegratorForce::computeIntegral(
    dynamicgraph::Vector& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector& dvel = velocityDerivativeSOUT(time);
  res = dvel;
  res *= timeStep;
  if (velocityPrecSIN) {
    const dynamicgraph::Vector& vel = velocityPrecSIN(time);
    res += vel;
  } else { /* vprec not set yet. */
  }
  velocityPrecSIN = res;

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Matrix& IntegratorForce::computeMassInverse(
    dynamicgraph::Matrix& res, const int& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix& mass = massSIN(time);
  res = mass.inverse();

  sotDEBUGOUT(15);
  return res;
}
