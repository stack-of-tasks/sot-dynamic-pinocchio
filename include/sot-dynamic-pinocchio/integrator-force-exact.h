/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic-pinocchio.
 * sot-dynamic-pinocchio is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic-pinocchio is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic-pinocchio.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__
#define __SOT_SOTINTEGRATORFORCEEXACT_H__

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

#include <sot-dynamic-pinocchio/integrator-force.h>

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


namespace dynamicgraph { namespace sot {
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
  dynamicgraph::Vector& computeVelocityExact( dynamicgraph::Vector& res,
				    const int& time );
  
/*  public: /\* --- PARAMS --- *\/ */
/*   virtual void commandLine( const std::string& cmdLine, */
/* 			    std::istringstream& cmdArgs, */
/* 			    std::ostream& os ); */
    

};


} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_SOTINTEGRATORFORCEEXACT_H__

