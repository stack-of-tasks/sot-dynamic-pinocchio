/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__
#define __SOT_SOTINTEGRATORFORCEEXACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/dynamic-pinocchio/integrator-force.h>

#include <sot/core/matrix-geometry.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(integrator_force_exact_EXPORTS)
#define SOTINTEGRATORFORCEEXACT_EXPORT __declspec(dllexport)
#else
#define SOTINTEGRATORFORCEEXACT_EXPORT __declspec(dllimport)
#endif
#else
#define SOTINTEGRATORFORCEEXACT_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCEEXACT_EXPORT IntegratorForceExact
    : public IntegratorForce {
 public:
  static const std::string CLASS_NAME;

 protected:
 public: /* --- CONSTRUCTION --- */
  IntegratorForceExact(const std::string& name);
  virtual ~IntegratorForceExact(void);

 public: /* --- SIGNAL --- */
 public: /* --- FUNCTIONS --- */
  dynamicgraph::Vector& computeVelocityExact(dynamicgraph::Vector& res,
                                             const sigtime_t& time);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__
