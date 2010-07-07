/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      IntegratorForceExact.h
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



#ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__
#define __SOT_SOTINTEGRATORFORCEEXACT_H__

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
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>
#include <sot-dynamic/integrator-force.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (integrator_force_exact_EXPORTS)
#    define SOTINTEGRATORFORCEEXACT_EXPORT __declspec(dllexport)
#  else  
#    define SOTINTEGRATORFORCEEXACT_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTINTEGRATORFORCEEXACT_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCEEXACT_EXPORT IntegratorForceExact
:public IntegratorForce
{
 public:
  static const std::string CLASS_NAME;

 protected:

 public: /* --- CONSTRUCTION --- */

  IntegratorForceExact( const std::string& name );
  virtual ~IntegratorForceExact( void );

 public: /* --- SIGNAL --- */


 public: /* --- FUNCTIONS --- */
  ml::Vector& computeVelocityExact( ml::Vector& res,
				    const int& time );
  
/*  public: /\* --- PARAMS --- *\/ */
/*   virtual void commandLine( const std::string& cmdLine, */
/* 			    std::istringstream& cmdArgs, */
/* 			    std::ostream& os ); */
    

};


} // namespace sot



#endif // #ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__

