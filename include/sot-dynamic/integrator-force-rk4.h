/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      IntegratorForceRK4.h
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



#ifndef __SOT_SOTINTEGRATORFORCERK4_H__
#define __SOT_SOTINTEGRATORFORCERK4_H__

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
#  if defined (integrator_force_rk4_EXPORTS)
#    define SOTINTEGRATORFORCERK4_EXPORT __declspec(dllexport)
#  else  
#    define SOTINTEGRATORFORCERK4_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTINTEGRATORFORCERK4_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCERK4_EXPORT IntegratorForceRK4
:public IntegratorForce
{
 public:
  static const std::string CLASS_NAME;

 protected:

 public: /* --- CONSTRUCTION --- */

  IntegratorForceRK4( const std::string& name );
  virtual ~IntegratorForceRK4( void );

 public: /* --- SIGNAL --- */


 public: /* --- FUNCTIONS --- */
  ml::Vector& computeDerivativeRK4( ml::Vector& res,
				    const int& time );
  
/*  public: /\* --- PARAMS --- *\/ */
/*   virtual void commandLine( const std::string& cmdLine, */
/* 			    std::istringstream& cmdArgs, */
/* 			    std::ostream& os ); */
    

};


} // namespace sot




#endif // #ifndef __SOT_SOTINTEGRATORFORCERK4_H__

