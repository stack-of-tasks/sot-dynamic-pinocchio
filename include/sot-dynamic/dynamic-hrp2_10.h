/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright JRL-Japan, 2010
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      DynamicHrp2_10.h
 * Project:   SOT
 * Author:    Olivier Stasse
 *            Nicolas Mansard
 *
 * For license see the file License.txt
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_DYNAMIC_HRP2_10_H__
#define __SOT_DYNAMIC_HRP2_10_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dynamic/dynamic.h>

/* JRL dynamic */
#include "hrp2-10-optimized/hrp2-10-small-optimized.hh"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (dynamic_hrp2_10_EXPORTS)
#    define DYNAMICHRP2_10_EXPORT __declspec(dllexport)
#  else  
#    define DYNAMICHRP2_10_EXPORT __declspec(dllimport)
#  endif 
#else
#  define DYNAMICHRP2_10_EXPORT
#endif

namespace sot {
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup signals
  \brief Modification of the classic dynamic class to get the
  optimized version of the kinematics computations.
*/
  
class DYNAMICHRP2_10_EXPORT DynamicHrp2_10
:public Dynamic
{

 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  DynamicHrp2_10( const std::string& name );
  virtual ~DynamicHrp2_10( void );

 public: /* --- MODEL CREATION --- */
 
   void buildModelHrp2( void );
  

};

} // namespace sot


#endif // #ifndef __SOT_DYNAMIC_HRP2_H__

