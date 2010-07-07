/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      AngleEstimator.h
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



#ifndef __SOT_ANGLE_ESTIMATOR_H__
#define __SOT_ANGLE_ESTIMATOR_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (angle_estimator_EXPORTS)
#    define SOTANGLEESTIMATOR_EXPORT __declspec(dllexport)
#  else  
#    define SOTANGLEESTIMATOR_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTANGLEESTIMATOR_EXPORT
#endif

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


namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTANGLEESTIMATOR_EXPORT AngleEstimator
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

 public: /* --- CONSTRUCTION --- */

  AngleEstimator( const std::string& name );
  virtual ~AngleEstimator( void );

 public: /* --- SIGNAL --- */

  dg::SignalPtr<MatrixRotation,int> sensorWorldRotationSIN; // estimate(worldRc)
  dg::SignalPtr<MatrixHomogeneous,int> sensorEmbeddedPositionSIN; // waistRchest
  dg::SignalPtr<MatrixHomogeneous,int> contactWorldPositionSIN; // estimate(worldRf)
  dg::SignalPtr<MatrixHomogeneous,int> contactEmbeddedPositionSIN; // waistRleg
  dg::SignalTimeDependent<ml::Vector,int> anglesSOUT;  // [ flex1 flex2 yaw_drift ]
  dg::SignalTimeDependent<MatrixRotation,int> flexibilitySOUT;  // footRleg
  dg::SignalTimeDependent<MatrixRotation,int> driftSOUT;  // Ryaw = worldRc est(wRc)^-1
  dg::SignalTimeDependent<MatrixRotation,int> sensorWorldRotationSOUT;  // worldRc
  dg::SignalTimeDependent<MatrixRotation,int> waistWorldRotationSOUT;  // worldRwaist
  dg::SignalTimeDependent<MatrixHomogeneous,int> waistWorldPositionSOUT; // worldMwaist
  dg::SignalTimeDependent<ml::Vector,int> waistWorldPoseRPYSOUT; // worldMwaist

 public: /* --- FUNCTIONS --- */
  ml::Vector& computeAngles( ml::Vector& res,
			     const int& time );
  MatrixRotation& computeFlexibilityFromAngles( MatrixRotation& res,
						   const int& time );
  MatrixRotation& computeDriftFromAngles( MatrixRotation& res,
					     const int& time );
  MatrixRotation& computeSensorWorldRotation( MatrixRotation& res,
						 const int& time ); 
  MatrixRotation& computeWaistWorldRotation( MatrixRotation& res,
						const int& time );
  MatrixHomogeneous& computeWaistWorldPosition( MatrixHomogeneous& res,
						   const int& time );
  ml::Vector& computeWaistWorldPoseRPY( ml::Vector& res,
					const int& time );
  
 public: /* --- PARAMS --- */
  bool fromSensor;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
    

};


} // namespace sot



#endif // #ifndef __SOT_ANGLE_ESTIMATOR_H__

