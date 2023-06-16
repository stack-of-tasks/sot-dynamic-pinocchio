/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTMASSAPPARENT_H__
#define __SOT_SOTMASSAPPARENT_H__

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
#if defined(mass_apparent_EXPORTS)
#define SOTMASSAPPARENT_EXPORT __declspec(dllexport)
#else
#define SOTMASSAPPARENT_EXPORT __declspec(dllimport)
#endif
#else
#define SOTMASSAPPARENT_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOTMASSAPPARENT_EXPORT MassApparent : public dg::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

 public: /* --- CONSTRUCTION --- */
  MassApparent(const std::string& name);
  virtual ~MassApparent(void);

 public: /* --- SIGNAL --- */
  dg::SignalPtr<dynamicgraph::Matrix, sigtime_t> jacobianSIN;
  dg::SignalPtr<dynamicgraph::Matrix, sigtime_t> inertiaInverseSIN;
  dg::SignalTimeDependent<dynamicgraph::Matrix, sigtime_t> massInverseSOUT;
  dg::SignalTimeDependent<dynamicgraph::Matrix, sigtime_t> massSOUT;

  dg::SignalPtr<dynamicgraph::Matrix, sigtime_t> inertiaSIN;
  dg::SignalTimeDependent<dynamicgraph::Matrix, sigtime_t> inertiaInverseSOUT;

 public: /* --- FUNCTIONS --- */
  dynamicgraph::Matrix& computeMassInverse(dynamicgraph::Matrix& res,
                                           const sigtime_t& time);
  dynamicgraph::Matrix& computeMass(dynamicgraph::Matrix& res, const sigtime_t& time);
  dynamicgraph::Matrix& computeInertiaInverse(dynamicgraph::Matrix& res,
                                              const sigtime_t& time);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_SOTMASSAPPARENT_H__
