/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTINTEGRATORFORCERK4_H__
#define __SOT_SOTINTEGRATORFORCERK4_H__

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

#include <sot/dynamic-pinocchio/integrator-force.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(integrator_force_rk4_EXPORTS)
#define SOTINTEGRATORFORCERK4_EXPORT __declspec(dllexport)
#else
#define SOTINTEGRATORFORCERK4_EXPORT __declspec(dllimport)
#endif
#else
#define SOTINTEGRATORFORCERK4_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCERK4_EXPORT IntegratorForceRK4 : public IntegratorForce {
 public:
  static const std::string CLASS_NAME;

 protected:
 public: /* --- CONSTRUCTION --- */
  IntegratorForceRK4(const std::string& name);
  virtual ~IntegratorForceRK4(void);

 public: /* --- SIGNAL --- */
 public: /* --- FUNCTIONS --- */
  dynamicgraph::Vector& computeDerivativeRK4(dynamicgraph::Vector& res, const int& time);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_SOTINTEGRATORFORCERK4_H__
