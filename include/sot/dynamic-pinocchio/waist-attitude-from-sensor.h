/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__
#define __SOT_WAISTATTITUDEFROMSENSOR_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/matrix-geometry.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(waist_attitude_from_sensor_EXPORTS)
#define SOTWAISTATTITUDEFROMSENSOR_EXPORT __declspec(dllexport)
#else
#define SOTWAISTATTITUDEFROMSENSOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTWAISTATTITUDEFROMSENSOR_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTWAISTATTITUDEFROMSENSOR_EXPORT WaistAttitudeFromSensor
    : public dg::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */
  WaistAttitudeFromSensor(const std::string& name);
  virtual ~WaistAttitudeFromSensor(void);

 public: /* --- SIGNAL --- */
  VectorRollPitchYaw& computeAttitudeWaist(VectorRollPitchYaw& res,
                                           const sigtime_t& time);

  dg::SignalPtr<MatrixRotation, sigtime_t> attitudeSensorSIN;
  dg::SignalPtr<MatrixHomogeneous, sigtime_t> positionSensorSIN;
  dg::SignalTimeDependent<VectorRollPitchYaw, sigtime_t> attitudeWaistSOUT;
};

class SOTWAISTATTITUDEFROMSENSOR_EXPORT WaistPoseFromSensorAndContact
    : public WaistAttitudeFromSensor {
 public:
  static const std::string CLASS_NAME;

 protected:
  void fromSensor(const bool& inFromSensor) { fromSensor_ = inFromSensor; }
  bool fromSensor() const { return fromSensor_; }

 private:
  bool fromSensor_;

 public: /* --- CONSTRUCTION --- */
  WaistPoseFromSensorAndContact(const std::string& name);
  virtual ~WaistPoseFromSensorAndContact(void);

 public: /* --- SIGNAL --- */
  dynamicgraph::Vector& computePositionWaist(dynamicgraph::Vector& res,
                                             const sigtime_t& time);

  dg::SignalPtr<MatrixHomogeneous, sigtime_t> positionContactSIN;
  dg::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> positionWaistSOUT;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__
