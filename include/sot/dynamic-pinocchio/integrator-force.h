/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTINTEGRATORFORCE_H__
#define __SOT_SOTINTEGRATORFORCE_H__

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
#if defined(integrator_force_EXPORTS)
#define SOTINTEGRATORFORCE_EXPORT __declspec(dllexport)
#else
#define SOTINTEGRATORFORCE_EXPORT __declspec(dllimport)
#endif
#else
#define SOTINTEGRATORFORCE_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCE_EXPORT IntegratorForce : public dg::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

 protected:
  double timeStep;
  static const double TIME_STEP_DEFAULT;  // = 5e-3

 public: /* --- CONSTRUCTION --- */
  IntegratorForce(const std::string& name);
  virtual ~IntegratorForce(void);

 public: /* --- SIGNAL --- */
  dg::SignalPtr<dynamicgraph::Vector, int> forceSIN;
  dg::SignalPtr<dynamicgraph::Matrix, int> massInverseSIN;
  dg::SignalPtr<dynamicgraph::Matrix, int> frictionSIN;

  /* Memory of the previous iteration. The sig is fed by the previous
   * computations. */
  dg::SignalPtr<dynamicgraph::Vector, int> velocityPrecSIN;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> velocityDerivativeSOUT;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> velocitySOUT;

  dg::SignalPtr<dynamicgraph::Matrix, int> massSIN;
  dg::SignalTimeDependent<dynamicgraph::Matrix, int> massInverseSOUT;

 public: /* --- FUNCTIONS --- */
  dynamicgraph::Vector& computeDerivative(dynamicgraph::Vector& res,
                                          const int& time);
  dynamicgraph::Vector& computeIntegral(dynamicgraph::Vector& res,
                                        const int& time);

  dynamicgraph::Matrix& computeMassInverse(dynamicgraph::Matrix& res,
                                           const int& time);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__
