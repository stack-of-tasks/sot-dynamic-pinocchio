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

#ifndef __SOT_SOTFORCECOMPENSATION_H__
#define __SOT_SOTFORCECOMPENSATION_H__

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
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-force.h>
#include <sot-core/matrix-homogeneous.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (force_compensation_EXPORTS)
#    define SOTFORCECOMPENSATION_EXPORT __declspec(dllexport)
#  else  
#    define SOTFORCECOMPENSATION_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFORCECOMPENSATION_EXPORT
#endif


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTFORCECOMPENSATION_EXPORT ForceCompensation
{
 private:
  static MatrixRotation I3;
 protected:
  bool usingPrecompensation;

 public:
  ForceCompensation( void );
  static MatrixForce& computeHandXworld( 
					   const MatrixRotation & worldRhand,
					   const ml::Vector & transSensorCom,
					   MatrixForce& res );

  
  static MatrixForce& computeHandVsensor( const MatrixRotation & sensorRhand,
					     MatrixForce& res );
  static MatrixForce& computeSensorXhand( const MatrixRotation & sensorRhand,
					     const ml::Vector & transSensorCom,
					     MatrixForce& res );
/*   static ml::Matrix& computeInertiaSensor( const ml::Matrix& inertiaJoint, */
/* 					   const MatrixForce& sensorXhand, */
/* 					   ml::Matrix& res ); */

  static ml::Vector& computeTorsorCompensated( const ml::Vector& torqueInput,
					       const ml::Vector& torquePrecompensation,
					       const ml::Vector& gravity,
					       const MatrixForce& handXworld,
					       const MatrixForce& handVsensor,
					       const ml::Matrix& gainSensor,
					       const ml::Vector& momentum,
					       ml::Vector& res );

  static ml::Vector& crossProduct_V_F( const ml::Vector& velocity,
				       const ml::Vector& force,
				       ml::Vector& res );
  static ml::Vector& computeMomentum( const ml::Vector& velocity,
				      const ml::Vector& acceleration,
				      const MatrixForce& sensorXhand,
				      const ml::Matrix& inertiaJoint,
				      ml::Vector& res );

  static ml::Vector& computeDeadZone( const ml::Vector& torqueInput,
				      const ml::Vector& deadZoneLimit,
				      ml::Vector& res );
  
 public: // CALIBRATION

  std::list<ml::Vector> torsorList;
  std::list<MatrixRotation> rotationList;

  void clearCalibration( void );
  void addCalibrationValue( const ml::Vector& torsor,
			    const MatrixRotation & worldRhand );
  
  ml::Vector calibrateTransSensorCom( const ml::Vector& gravity,
				      const MatrixRotation& handRsensor );
  ml::Vector calibrateGravity( const MatrixRotation& handRsensor,
			       bool precompensationCalibration = false,
			       const MatrixRotation& hand0Rsensor = I3 );

    
  
};

/* --------------------------------------------------------------------- */
/* --- PLUGIN ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTFORCECOMPENSATION_EXPORT ForceCompensationPlugin
:public dg::Entity, public ForceCompensation
{
 public:
  static const std::string CLASS_NAME;
  bool calibrationStarted;


 public: /* --- CONSTRUCTION --- */

  ForceCompensationPlugin( const std::string& name );
  virtual ~ForceCompensationPlugin( void );

 public: /* --- SIGNAL --- */

  /* --- INPUTS --- */
  dg::SignalPtr<ml::Vector,int> torsorSIN; 
  dg::SignalPtr<MatrixRotation,int> worldRhandSIN; 

  /* --- CONSTANTS --- */
  dg::SignalPtr<MatrixRotation,int> handRsensorSIN; 
  dg::SignalPtr<ml::Vector,int> translationSensorComSIN; 
  dg::SignalPtr<ml::Vector,int> gravitySIN; 
  dg::SignalPtr<ml::Vector,int> precompensationSIN; 
  dg::SignalPtr<ml::Matrix,int> gainSensorSIN; 
  dg::SignalPtr<ml::Vector,int> deadZoneLimitSIN; 
  dg::SignalPtr<ml::Vector,int> transSensorJointSIN; 
  dg::SignalPtr<ml::Matrix,int> inertiaJointSIN; 

  dg::SignalPtr<ml::Vector,int> velocitySIN; 
  dg::SignalPtr<ml::Vector,int> accelerationSIN; 

  /* --- INTERMEDIATE OUTPUTS --- */
  dg::SignalTimeDependent<MatrixForce,int> handXworldSOUT; 
  dg::SignalTimeDependent<MatrixForce,int> handVsensorSOUT; 
  dg::SignalPtr<ml::Vector,int> torsorDeadZoneSIN; 

  dg::SignalTimeDependent<MatrixForce,int> sensorXhandSOUT;
  //dg::SignalTimeDependent<ml::Matrix,int> inertiaSensorSOUT;
  dg::SignalTimeDependent<ml::Vector,int> momentumSOUT; 
  dg::SignalPtr<ml::Vector,int> momentumSIN; 

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<ml::Vector,int> torsorCompensatedSOUT; 
  dg::SignalTimeDependent<ml::Vector,int> torsorDeadZoneSOUT;

  typedef int sotDummyType;
  dg::SignalTimeDependent<sotDummyType,int> calibrationTrigerSOUT; 

 public: /* --- COMMANDLINE --- */

  sotDummyType& calibrationTriger( sotDummyType& dummy,int time );


  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );



};


} // namaspace sot



#endif // #ifndef __SOT_SOTFORCECOMPENSATION_H__

