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

#ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__
#define __SOT_WAISTATTITUDEFROMSENSOR_H__

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
#include <sot-core/vector-roll-pitch-yaw.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (waist_attitude_from_sensor_EXPORTS)
#    define SOTWAISTATTITUDEFROMSENSOR_EXPORT __declspec(dllexport)
#  else  
#    define SOTWAISTATTITUDEFROMSENSOR_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTWAISTATTITUDEFROMSENSOR_EXPORT
#endif


namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTWAISTATTITUDEFROMSENSOR_EXPORT WaistAttitudeFromSensor
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  WaistAttitudeFromSensor( const std::string& name );
  virtual ~WaistAttitudeFromSensor( void );

 public: /* --- SIGNAL --- */

  VectorRollPitchYaw & computeAttitudeWaist( VectorRollPitchYaw & res,
					       const int& time );

  dg::SignalPtr<MatrixRotation,int> attitudeSensorSIN;
  dg::SignalPtr<MatrixHomogeneous,int> positionSensorSIN;
  dg::SignalTimeDependent<VectorRollPitchYaw,int> attitudeWaistSOUT;


 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


class SOTWAISTATTITUDEFROMSENSOR_EXPORT WaistPoseFromSensorAndContact
:public WaistAttitudeFromSensor
{
 public:
  static const std::string CLASS_NAME;

 protected:
  void fromSensor(const bool& inFromSensor) {
    fromSensor_ = inFromSensor;
  }
  bool fromSensor() const {
    return fromSensor_;
  }
 private:
  bool fromSensor_;

 public: /* --- CONSTRUCTION --- */

  WaistPoseFromSensorAndContact( const std::string& name );
  virtual ~WaistPoseFromSensorAndContact( void );

 public: /* --- SIGNAL --- */

  ml::Vector& computePositionWaist( ml::Vector& res,
				    const int& time );

  dg::SignalPtr<MatrixHomogeneous,int> positionContactSIN;
  dg::SignalTimeDependent<ml::Vector,int> positionWaistSOUT;


 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__

