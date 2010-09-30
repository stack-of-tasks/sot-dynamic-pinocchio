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

#ifndef __SOT_SOTINTEGRATORFORCE_H__
#define __SOT_SOTINTEGRATORFORCE_H__

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
#  if defined (integrator_force_EXPORTS)
#    define SOTINTEGRATORFORCE_EXPORT __declspec(dllexport)
#  else  
#    define SOTINTEGRATORFORCE_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTINTEGRATORFORCE_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTINTEGRATORFORCE_EXPORT IntegratorForce
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 protected:
  double timeStep;
  static const double  TIME_STEP_DEFAULT ; // = 5e-3


 public: /* --- CONSTRUCTION --- */

  IntegratorForce( const std::string& name );
  virtual ~IntegratorForce( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<ml::Vector,int> forceSIN; 
  dg::SignalPtr<ml::Matrix,int> massInverseSIN; 
  dg::SignalPtr<ml::Matrix,int> frictionSIN; 

  /* Memory of the previous iteration. The sig is fed by the previous
   * computations. */
  dg::SignalPtr<ml::Vector,int> velocityPrecSIN; 
  dg::SignalTimeDependent<ml::Vector,int> velocityDerivativeSOUT; 
  dg::SignalTimeDependent<ml::Vector,int> velocitySOUT; 

  dg::SignalPtr<ml::Matrix,int> massSIN; 
  dg::SignalTimeDependent<ml::Matrix,int> massInverseSOUT; 

 public: /* --- FUNCTIONS --- */
  ml::Vector& computeDerivative( ml::Vector& res,
				  const int& time );
  ml::Vector& computeIntegral( ml::Vector& res,
			       const int& time );

  ml::Matrix& computeMassInverse( ml::Matrix& res,
				  const int& time );

  
 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} // namespace sot



#endif // #ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__

