/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      MassApparent.h
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



#ifndef __SOT_SOTMASSAPPARENT_H__
#define __SOT_SOTMASSAPPARENT_H__

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

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (mass_apparent_EXPORTS)
#    define SOTMASSAPPARENT_EXPORT __declspec(dllexport)
#  else  
#    define SOTMASSAPPARENT_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTMASSAPPARENT_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOTMASSAPPARENT_EXPORT MassApparent
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  MassApparent( const std::string& name );
  virtual ~MassApparent( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<ml::Matrix,int> jacobianSIN; 
  dg::SignalPtr<ml::Matrix,int> inertiaInverseSIN; 
  dg::SignalTimeDependent<ml::Matrix,int> massInverseSOUT; 
  dg::SignalTimeDependent<ml::Matrix,int> massSOUT; 

  dg::SignalPtr<ml::Matrix,int> inertiaSIN; 
  dg::SignalTimeDependent<ml::Matrix,int> inertiaInverseSOUT; 

 public: /* --- FUNCTIONS --- */
  ml::Matrix& computeMassInverse( ml::Matrix& res,const int& time );
  ml::Matrix& computeMass( ml::Matrix& res,const int& time );
  ml::Matrix& computeInertiaInverse( ml::Matrix& res,const int& time );
};


}


#endif // #ifndef __SOT_SOTMASSAPPARENT_H__

