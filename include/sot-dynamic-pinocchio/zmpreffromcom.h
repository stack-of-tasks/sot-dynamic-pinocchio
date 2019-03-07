/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_ZMPREFFROMCOM_H__
#define __SOT_ZMPREFFROMCOM_H__

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

#if defined (WIN32)
#  if defined (zmpreffromcom_EXPORTS)
#    define SOTZMPREFFROMCOM_EXPORT __declspec(dllexport)
#  else
#    define SOTZMPREFFROMCOM_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTZMPREFFROMCOM_EXPORT
#endif

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTZMPREFFROMCOM_EXPORT ZmprefFromCom
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
 public:
  double dt;
  const static double DT_DEFAULT; // = 5e-3; // 5ms
  double footHeight;
  const static double FOOT_HEIGHT_DEFAULT; // = .105;

 public: /* --- CONSTRUCTION --- */

  ZmprefFromCom( const std::string& name );
  virtual ~ZmprefFromCom( void );

 public: /* --- SIGNAL --- */

  dynamicgraph::Vector& computeZmpref( dynamicgraph::Vector& res,
					       const int& time );

  dg::SignalPtr<MatrixHomogeneous,int> waistPositionSIN;
  dg::SignalPtr<dynamicgraph::Vector,int> comPositionSIN;
  dg::SignalPtr<dynamicgraph::Vector,int> dcomSIN;
  dg::SignalTimeDependent<dynamicgraph::Vector,int> zmprefSOUT;

};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_ZMPREFFROMCOM_H__
