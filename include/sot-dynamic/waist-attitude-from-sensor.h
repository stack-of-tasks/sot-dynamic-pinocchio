/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      WaistAttitudeFromSensor.h
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



#ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__
#define __SOT_WAISTATTITUDEFROMSENSOR_H__

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


namespace sot {
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
  bool fromSensor;

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


} // namespace sot


#endif // #ifndef __SOT_WAISTATTITUDEFROMSENSOR_H__

