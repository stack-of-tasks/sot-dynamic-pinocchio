/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic.
 * sot-dynamic is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_DYNAMIC_HRP2_10_old_H__
#define __SOT_DYNAMIC_HRP2_10_old_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dynamic/dynamic.h>

/* JRL dynamic */
#include "hrp2-10-optimized/hrp2-10-small-old-optimized.hh"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (dynamic_hrp2_10_old_EXPORTS)
#    define DynamicHrp2_10_old_EXPORT __declspec(dllexport)
#  else  
#    define DynamicHrp2_10_old_EXPORT __declspec(dllimport)
#  endif 
#else
#  define DynamicHrp2_10_old_EXPORT
#endif

namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup signals
  \brief Modification of the classic dynamic class to get the
  optimized version of the kinematics computations.
*/
  
class DynamicHrp2_10_old_EXPORT DynamicHrp2_10_old
:public Dynamic
{

 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  DynamicHrp2_10_old( const std::string& name );
  virtual ~DynamicHrp2_10_old( void );

 public: /* --- MODEL CREATION --- */
 
   void buildModelHrp2( void );
  

};


} // namespace sot


#endif // #ifndef __SOT_DYNAMIC_HRP2_H__

