/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_ANGLE_ESTIMATOR_H__
#define __SOT_ANGLE_ESTIMATOR_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(angle_estimator_EXPORTS)
#define SOTANGLEESTIMATOR_EXPORT __declspec(dllexport)
#else
#define SOTANGLEESTIMATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTANGLEESTIMATOR_EXPORT
#endif

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

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTANGLEESTIMATOR_EXPORT AngleEstimator : public dg::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */
  AngleEstimator(const std::string& name);
  virtual ~AngleEstimator(void);

 public: /* --- SIGNAL --- */
  dg::SignalPtr<MatrixRotation, sigtime_t>
      sensorWorldRotationSIN;  // estimate(worldRc)
  dg::SignalPtr<MatrixHomogeneous, sigtime_t>
      sensorEmbeddedPositionSIN;  // waistRchest
  dg::SignalPtr<MatrixHomogeneous, sigtime_t>
      contactWorldPositionSIN;  // estimate(worldRf)
  dg::SignalPtr<MatrixHomogeneous, sigtime_t>
      contactEmbeddedPositionSIN;  // waistRleg
  dg::SignalTimeDependent<dynamicgraph::Vector, sigtime_t>
      anglesSOUT;  // [ flex1 flex2 yaw_drift ]
  dg::SignalTimeDependent<MatrixRotation, sigtime_t> flexibilitySOUT;  // footRleg
  dg::SignalTimeDependent<MatrixRotation, sigtime_t>
      driftSOUT;  // Ryaw = worldRc est(wRc)^-1
  dg::SignalTimeDependent<MatrixRotation, sigtime_t>
      sensorWorldRotationSOUT;  // worldRc
  dg::SignalTimeDependent<MatrixRotation, sigtime_t>
      waistWorldRotationSOUT;  // worldRwaist
  dg::SignalTimeDependent<MatrixHomogeneous, sigtime_t>
      waistWorldPositionSOUT;  // worldMwaist
  dg::SignalTimeDependent<dynamicgraph::Vector, sigtime_t>
      waistWorldPoseRPYSOUT;  // worldMwaist

  dg::SignalPtr<dynamicgraph::Matrix, sigtime_t> jacobianSIN;
  dg::SignalPtr<dynamicgraph::Vector, sigtime_t> qdotSIN;
  dg::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> xff_dotSOUT;
  dg::SignalTimeDependent<dynamicgraph::Vector, sigtime_t> qdotSOUT;

 public: /* --- FUNCTIONS --- */
  dynamicgraph::Vector& computeAngles(dynamicgraph::Vector& res,
                                      const sigtime_t& time);
  MatrixRotation& computeFlexibilityFromAngles(MatrixRotation& res,
                                               const sigtime_t& time);
  MatrixRotation& computeDriftFromAngles(MatrixRotation& res, const sigtime_t& time);
  MatrixRotation& computeSensorWorldRotation(MatrixRotation& res,
                                             const sigtime_t& time);
  MatrixRotation& computeWaistWorldRotation(MatrixRotation& res,
                                            const sigtime_t& time);
  MatrixHomogeneous& computeWaistWorldPosition(MatrixHomogeneous& res,
                                               const sigtime_t& time);
  dynamicgraph::Vector& computeWaistWorldPoseRPY(dynamicgraph::Vector& res,
                                                 const sigtime_t& time);
  dynamicgraph::Vector& compute_xff_dotSOUT(dynamicgraph::Vector& res,
                                            const sigtime_t& time);
  dynamicgraph::Vector& compute_qdotSOUT(dynamicgraph::Vector& res,
                                         const sigtime_t& time);

 public: /* --- PARAMS --- */
  void fromSensor(const bool& fs) { fromSensor_ = fs; }
  bool fromSensor() const { return fromSensor_; }

 private:
  bool fromSensor_;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_ANGLE_ESTIMATOR_H__
