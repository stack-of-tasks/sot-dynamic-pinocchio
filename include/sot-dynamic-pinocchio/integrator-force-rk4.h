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

#ifndef __SOT_SOTINTEGRATORFORCERK4_H__
#define __SOT_SOTINTEGRATORFORCERK4_H__

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


namespace dynamicgraph { namespace sot {
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
  dynamicgraph::Vector& computeDerivativeRK4( dynamicgraph::Vector& res,
				    const int& time );
  
/*  public: /\* --- PARAMS --- *\/ */
/*   virtual void commandLine( const std::string& cmdLine, */
/* 			    std::istringstream& cmdArgs, */
/* 			    std::ostream& os ); */
    

};


} /* namespace sot */} /* namespace dynamicgraph */




#endif // #ifndef __SOT_SOTINTEGRATORFORCERK4_H__

