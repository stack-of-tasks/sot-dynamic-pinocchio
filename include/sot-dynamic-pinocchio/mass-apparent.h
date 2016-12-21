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

#ifndef __SOT_SOTMASSAPPARENT_H__
#define __SOT_SOTMASSAPPARENT_H__

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


namespace dynamicgraph { namespace sot {
    namespace dg = dynamicgraph;

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    class SOTMASSAPPARENT_EXPORT MassApparent
      :public dg::Entity
      {
      public:
	static const std::string CLASS_NAME;
	virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

      public: /* --- CONSTRUCTION --- */

	MassApparent( const std::string& name );
	virtual ~MassApparent( void );

      public: /* --- SIGNAL --- */

	dg::SignalPtr<dynamicgraph::Matrix,int> jacobianSIN; 
	dg::SignalPtr<dynamicgraph::Matrix,int> inertiaInverseSIN; 
	dg::SignalTimeDependent<dynamicgraph::Matrix,int> massInverseSOUT; 
	dg::SignalTimeDependent<dynamicgraph::Matrix,int> massSOUT; 

	dg::SignalPtr<dynamicgraph::Matrix,int> inertiaSIN; 
	dg::SignalTimeDependent<dynamicgraph::Matrix,int> inertiaInverseSOUT; 

      public: /* --- FUNCTIONS --- */
	dynamicgraph::Matrix& computeMassInverse( dynamicgraph::Matrix& res,const int& time );
	dynamicgraph::Matrix& computeMass( dynamicgraph::Matrix& res,const int& time );
	dynamicgraph::Matrix& computeInertiaInverse( dynamicgraph::Matrix& res,const int& time );
      };


  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_SOTMASSAPPARENT_H__

