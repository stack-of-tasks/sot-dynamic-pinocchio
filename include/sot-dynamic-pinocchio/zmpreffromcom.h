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

#ifndef __SOT_ZMPREFFROMCOM_H__
#define __SOT_ZMPREFFROMCOM_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (zmpreffromcom_EXPORTS)
#    define SOTZMPREFFROMCOM_EXPORT __declspec(dllexport)
#  else  
#    define SOTZMPREFFROMCOM_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTZMPREFFROMCOM_EXPORT
#endif

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTZMPREFFROMCOM_EXPORT ZmprefFromCom
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
 public:
  double dt;
  const static double DT_DEFAULT; // = 5e-3; // 5ms
  double footHeight;
  const static double FOOT_HEIGHT_DEFAULT; // = .105; 

 public: /* --- CONSTRUCTION --- */

  ZmprefFromCom( const std::string& name );
  virtual ~ZmprefFromCom( void );

 public: /* --- SIGNAL --- */

  ml::Vector& computeZmpref( ml::Vector& res,
					       const int& time );

  dg::SignalPtr<MatrixHomogeneous,int> waistPositionSIN;
  dg::SignalPtr<ml::Vector,int> comPositionSIN;
  dg::SignalPtr<ml::Vector,int> dcomSIN;
  dg::SignalTimeDependent<ml::Vector,int> zmprefSOUT;


 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_ZMPREFFROMCOM_H__

