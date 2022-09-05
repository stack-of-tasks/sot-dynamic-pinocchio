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
  dg::SignalPtr<MatrixRotation, int>
      sensorWorldRotationSIN;  // estimate(worldRc)
  dg::SignalPtr<MatrixHomogeneous, int>
      sensorEmbeddedPositionSIN;  // waistRchest
  dg::SignalPtr<MatrixHomogeneous, int>
      contactWorldPositionSIN;  // estimate(worldRf)
  dg::SignalPtr<MatrixHomogeneous, int>
      contactEmbeddedPositionSIN;  // waistRleg
  dg::SignalTimeDependent<dynamicgraph::Vector, int>
      anglesSOUT;  // [ flex1 flex2 yaw_drift ]
  dg::SignalTimeDependent<MatrixRotation, int> flexibilitySOUT;  // footRleg
  dg::SignalTimeDependent<MatrixRotation, int>
      driftSOUT;  // Ryaw = worldRc est(wRc)^-1
  dg::SignalTimeDependent<MatrixRotation, int>
      sensorWorldRotationSOUT;  // worldRc
  dg::SignalTimeDependent<MatrixRotation, int>
      waistWorldRotationSOUT;  // worldRwaist
  dg::SignalTimeDependent<MatrixHomogeneous, int>
      waistWorldPositionSOUT;  // worldMwaist
  dg::SignalTimeDependent<dynamicgraph::Vector, int>
      waistWorldPoseRPYSOUT;  // worldMwaist

  dg::SignalPtr<dynamicgraph::Matrix, int> jacobianSIN;
  dg::SignalPtr<dynamicgraph::Vector, int> qdotSIN;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> xff_dotSOUT;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> qdotSOUT;

 public: /* --- FUNCTIONS --- */
  dynamicgraph::Vector& computeAngles(dynamicgraph::Vector& res,
                                      const int& time);
  MatrixRotation& computeFlexibilityFromAngles(MatrixRotation& res,
                                               const int& time);
  MatrixRotation& computeDriftFromAngles(MatrixRotation& res, const int& time);
  MatrixRotation& computeSensorWorldRotation(MatrixRotation& res,
                                             const int& time);
  MatrixRotation& computeWaistWorldRotation(MatrixRotation& res,
                                            const int& time);
  MatrixHomogeneous& computeWaistWorldPosition(MatrixHomogeneous& res,
                                               const int& time);
  dynamicgraph::Vector& computeWaistWorldPoseRPY(dynamicgraph::Vector& res,
                                                 const int& time);
  dynamicgraph::Vector& compute_xff_dotSOUT(dynamicgraph::Vector& res,
                                            const int& time);
  dynamicgraph::Vector& compute_qdotSOUT(dynamicgraph::Vector& res,
                                         const int& time);

 public: /* --- PARAMS --- */
  void fromSensor(const bool& fs) { fromSensor_ = fs; }
  bool fromSensor() const { return fromSensor_; }

 private:
  bool fromSensor_;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_ANGLE_ESTIMATOR_H__
