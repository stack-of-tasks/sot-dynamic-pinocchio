/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/factory.h>
#include <sot/dynamic-pinocchio/zmpreffromcom.h>

#include <sot/core/debug.hh>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ZmprefFromCom, "ZmprefFromCom");

const double ZmprefFromCom::DT_DEFAULT = 5e-3;
const double ZmprefFromCom::FOOT_HEIGHT_DEFAULT = .105;

ZmprefFromCom::ZmprefFromCom(const std::string& name)
    : Entity(name),
      dt(DT_DEFAULT),
      footHeight(FOOT_HEIGHT_DEFAULT),
      waistPositionSIN(
          NULL, "sotZmprefFromCom(" + name + ")::input(MatrixHomo)::waist"),
      comPositionSIN(NULL,
                     "sotZmprefFromCom(" + name + ")::input(Vector)::com"),
      dcomSIN(NULL, "sotZmprefFromCom(" + name + ")::input(Vector)::dcom"),
      zmprefSOUT(boost::bind(&ZmprefFromCom::computeZmpref, this, _1, _2),
                 waistPositionSIN << comPositionSIN << dcomSIN,
                 "sotZmprefFromCom(" + name + ")::output(RPY)::zmpref") {
  sotDEBUGIN(5);

  signalRegistration(waistPositionSIN);
  signalRegistration(comPositionSIN);
  signalRegistration(dcomSIN);
  signalRegistration(zmprefSOUT);

  sotDEBUGOUT(5);
}

ZmprefFromCom::~ZmprefFromCom(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
dynamicgraph::Vector& ZmprefFromCom::computeZmpref(dynamicgraph::Vector& res,
                                                   const sigtime_t& time) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector& com = comPositionSIN(time);
  const dynamicgraph::Vector& dcom = dcomSIN(time);
  const MatrixHomogeneous& oTw = waistPositionSIN(time);

  MatrixHomogeneous wTo = oTw.inverse();
  dynamicgraph::Vector nextComRef = dcom;
  nextComRef *= dt;
  nextComRef += com;

  nextComRef(2) = -footHeight;  // projection on the ground.
  res = wTo.matrix() * nextComRef;

  sotDEBUGOUT(15);
  return res;
}
