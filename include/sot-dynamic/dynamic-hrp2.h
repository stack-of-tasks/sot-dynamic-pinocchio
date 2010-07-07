/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Dynamic.h
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



#ifndef __SOT_DYNAMIC_HRP2_H__
#define __SOT_DYNAMIC_HRP2_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dynamic/dynamic.h>

/* JRL dynamic */
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (dynamic-hrp2_EXPORTS)
#    define SOTDYNAMICHRP2_EXPORT __declspec(dllexport)
#  else  
#    define SOTDYNAMICHRP2_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTDYNAMICHRP2_EXPORT
#endif



namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup signals
  \brief Modification of the classic dynamic class to get the
  optimized version of the kinematics computations.
*/
  
class SOTDYNAMICHRP2_EXPORT DynamicHrp2
:public Dynamic
{

 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  DynamicHrp2( const std::string& name );
  virtual ~DynamicHrp2( void );

 public: /* --- MODEL CREATION --- */
 
   void buildModelHrp2( void );
  

};


} // namespace sot



#endif // #ifndef __SOT_DYNAMIC_HRP2_H__

