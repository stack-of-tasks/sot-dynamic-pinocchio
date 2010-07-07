/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      ZmprefFromCom.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_ZMPREFFROMCOM_H__
#define __SOT_ZMPREFFROMCOM_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot-core/matrix-homogeneous.h>

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

namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTZMPREFFROMCOM_EXPORT ZmprefFromCom
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  double dt;
  const static double DT_DEFAULT; // = 5e-3; // 5ms
  double footHeight;
  const static double FOOT_HEIGHT_DEFAULT; // = .105; 

 public: /* --- CONSTRUCTION --- */

  ZmprefFromCom( const std::string& name );
  virtual ~ZmprefFromCom( void );

 public: /* --- SIGNAL --- */

  ml::Vector& computeZmpref( ml::Vector& res,
					       const int& time );

  dg::SignalPtr<MatrixHomogeneous,int> waistPositionSIN;
  dg::SignalPtr<ml::Vector,int> comPositionSIN;
  dg::SignalPtr<ml::Vector,int> dcomSIN;
  dg::SignalTimeDependent<ml::Vector,int> zmprefSOUT;


 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} // namespace sot


#endif // #ifndef __SOT_ZMPREFFROMCOM_H__

