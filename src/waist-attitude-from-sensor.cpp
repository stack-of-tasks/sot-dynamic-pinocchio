/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/factory.h>
#include <sot/dynamic-pinocchio/waist-attitude-from-sensor.h>

#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistAttitudeFromSensor,
                                   "WaistAttitudeFromSensor");

WaistAttitudeFromSensor::WaistAttitudeFromSensor(const std::string& name)
    : Entity(name),
      attitudeSensorSIN(NULL, "sotWaistAttitudeFromSensor(" + name +
                                  ")::input(MatrixRotation)::attitudeIN"),
      positionSensorSIN(NULL, "sotWaistAttitudeFromSensor(" + name +
                                  ")::input(matrixHomo)::position"),
      attitudeWaistSOUT(
          boost::bind(&WaistAttitudeFromSensor::computeAttitudeWaist, this, _1,
                      _2),
          attitudeSensorSIN << positionSensorSIN,
          "sotWaistAttitudeFromSensor(" + name + ")::output(RPY)::attitude") {
  sotDEBUGIN(5);

  signalRegistration(attitudeSensorSIN);
  signalRegistration(positionSensorSIN);
  signalRegistration(attitudeWaistSOUT);

  sotDEBUGOUT(5);
}

WaistAttitudeFromSensor::~WaistAttitudeFromSensor(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
VectorRollPitchYaw& WaistAttitudeFromSensor::computeAttitudeWaist(
    VectorRollPitchYaw& res, const sigtime_t& time) {
  sotDEBUGIN(15);

  const MatrixHomogeneous& waistMchest = positionSensorSIN(time);
  const MatrixRotation& worldRchest = attitudeSensorSIN(time);

  MatrixRotation waistRchest;
  waistRchest = waistMchest.linear();
  MatrixRotation chestRwaist;
  chestRwaist = waistRchest.transpose();

  MatrixRotation worldrchest;
  worldrchest = worldRchest * chestRwaist;
  res = (worldrchest.eulerAngles(2, 1, 0)).reverse();
  sotDEBUGOUT(15);
  return res;
}

/* === WaistPoseFromSensorAndContact ===================================== */
/* === WaistPoseFromSensorAndContact ===================================== */
/* === WaistPoseFromSensorAndContact ===================================== */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistPoseFromSensorAndContact,
                                   "WaistPoseFromSensorAndContact");

WaistPoseFromSensorAndContact::WaistPoseFromSensorAndContact(
    const std::string& name)
    : WaistAttitudeFromSensor(name),
      fromSensor_(false),
      positionContactSIN(NULL, "sotWaistPoseFromSensorAndContact(" + name +
                                   ")::input(matrixHomo)::contact"),
      positionWaistSOUT(
          boost::bind(&WaistPoseFromSensorAndContact::computePositionWaist,
                      this, _1, _2),
          attitudeWaistSOUT << positionContactSIN,
          "sotWaistPoseFromSensorAndContact(" + name +
              ")::output(RPY+T)::positionWaist") {
  sotDEBUGIN(5);

  signalRegistration(positionContactSIN);
  signalRegistration(positionWaistSOUT);

  // Commands
  std::string docstring;
  docstring =
      "    \n"
      "    Set flag specifying whether angles are measured from sensors or "
      "simulated.\n"
      "    \n"
      "      Input:\n"
      "        - a boolean value.\n"
      "    \n";
  addCommand(
      "setFromSensor",
      new ::dynamicgraph::command::Setter<WaistPoseFromSensorAndContact, bool>(
          *this, &WaistPoseFromSensorAndContact::fromSensor, docstring));

  docstring =
      "    \n"
      "    Get flag specifying whether angles are measured from sensors or "
      "simulated.\n"
      "    \n"
      "      No input,\n"
      "      return a boolean value.\n"
      "    \n";
  addCommand(
      "getFromSensor",
      new ::dynamicgraph::command::Getter<WaistPoseFromSensorAndContact, bool>(
          *this, &WaistPoseFromSensorAndContact::fromSensor, docstring));

  sotDEBUGOUT(5);
}

WaistPoseFromSensorAndContact::~WaistPoseFromSensorAndContact(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
dynamicgraph::Vector& WaistPoseFromSensorAndContact::computePositionWaist(
    dynamicgraph::Vector& res, const sigtime_t& time) {
  sotDEBUGIN(15);

  const MatrixHomogeneous& waistMcontact = positionContactSIN(time);
  MatrixHomogeneous contactMwaist;
  contactMwaist = waistMcontact.inverse();

  res.resize(6);
  for (unsigned int i = 0; i < 3; ++i) {
    res(i) = contactMwaist(i, 3);
  }

  if (fromSensor_) {
    const VectorRollPitchYaw& worldrwaist = attitudeWaistSOUT(time);
    for (unsigned int i = 0; i < 3; ++i) {
      res(i + 3) = worldrwaist(i);
    }
  } else {
    VectorRollPitchYaw contactrwaist;
    contactrwaist = contactMwaist.linear().eulerAngles(2, 1, 0).reverse();

    for (unsigned int i = 0; i < 3; ++i) {
      res(i + 3) = contactrwaist(i);
    }
  }

  sotDEBUGOUT(15);
  return res;
}
