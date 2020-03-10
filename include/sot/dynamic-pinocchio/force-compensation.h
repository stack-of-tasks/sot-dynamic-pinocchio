/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTFORCECOMPENSATION_H__
#define __SOT_SOTFORCECOMPENSATION_H__

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

#if defined(WIN32)
#if defined(force_compensation_EXPORTS)
#define SOTFORCECOMPENSATION_EXPORT __declspec(dllexport)
#else
#define SOTFORCECOMPENSATION_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFORCECOMPENSATION_EXPORT
#endif

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTFORCECOMPENSATION_EXPORT ForceCompensation {
 private:
  static MatrixRotation I3;

 protected:
  bool usingPrecompensation;

 public:
  ForceCompensation(void);
  static MatrixForce& computeHandXworld(const MatrixRotation& worldRhand, const dynamicgraph::Vector& transSensorCom,
                                        MatrixForce& res);

  static MatrixForce& computeHandVsensor(const MatrixRotation& sensorRhand, MatrixForce& res);
  static MatrixForce& computeSensorXhand(const MatrixRotation& sensorRhand, const dynamicgraph::Vector& transSensorCom,
                                         MatrixForce& res);
  /*   static dynamicgraph::Matrix& computeInertiaSensor( const dynamicgraph::Matrix& inertiaJoint, */
  /* 					   const MatrixForce& sensorXhand, */
  /* 					   dynamicgraph::Matrix& res ); */

  static dynamicgraph::Vector& computeTorsorCompensated(
      const dynamicgraph::Vector& torqueInput, const dynamicgraph::Vector& torquePrecompensation,
      const dynamicgraph::Vector& gravity, const MatrixForce& handXworld, const MatrixForce& handVsensor,
      const dynamicgraph::Matrix& gainSensor, const dynamicgraph::Vector& momentum, dynamicgraph::Vector& res);

  static dynamicgraph::Vector& crossProduct_V_F(const dynamicgraph::Vector& velocity,
                                                const dynamicgraph::Vector& force, dynamicgraph::Vector& res);
  static dynamicgraph::Vector& computeMomentum(const dynamicgraph::Vector& velocity,
                                               const dynamicgraph::Vector& acceleration,
                                               const MatrixForce& sensorXhand,
                                               const dynamicgraph::Matrix& inertiaJoint, dynamicgraph::Vector& res);

  static dynamicgraph::Vector& computeDeadZone(const dynamicgraph::Vector& torqueInput,
                                               const dynamicgraph::Vector& deadZoneLimit, dynamicgraph::Vector& res);

 public:  // CALIBRATION
  std::list<dynamicgraph::Vector> torsorList;
  std::list<MatrixRotation> rotationList;

  void clearCalibration(void);
  void addCalibrationValue(const dynamicgraph::Vector& torsor, const MatrixRotation& worldRhand);

  dynamicgraph::Vector calibrateTransSensorCom(const dynamicgraph::Vector& gravity, const MatrixRotation& handRsensor);
  dynamicgraph::Vector calibrateGravity(const MatrixRotation& handRsensor, bool precompensationCalibration = false,
                                        const MatrixRotation& hand0Rsensor = I3);
};

/* --------------------------------------------------------------------- */
/* --- PLUGIN ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTFORCECOMPENSATION_EXPORT ForceCompensationPlugin : public dg::Entity, public ForceCompensation {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }
  bool calibrationStarted;

 public: /* --- CONSTRUCTION --- */
  ForceCompensationPlugin(const std::string& name);
  virtual ~ForceCompensationPlugin(void);

 public: /* --- SIGNAL --- */
  /* --- INPUTS --- */
  dg::SignalPtr<dynamicgraph::Vector, int> torsorSIN;
  dg::SignalPtr<MatrixRotation, int> worldRhandSIN;

  /* --- CONSTANTS --- */
  dg::SignalPtr<MatrixRotation, int> handRsensorSIN;
  dg::SignalPtr<dynamicgraph::Vector, int> translationSensorComSIN;
  dg::SignalPtr<dynamicgraph::Vector, int> gravitySIN;
  dg::SignalPtr<dynamicgraph::Vector, int> precompensationSIN;
  dg::SignalPtr<dynamicgraph::Matrix, int> gainSensorSIN;
  dg::SignalPtr<dynamicgraph::Vector, int> deadZoneLimitSIN;
  dg::SignalPtr<dynamicgraph::Vector, int> transSensorJointSIN;
  dg::SignalPtr<dynamicgraph::Matrix, int> inertiaJointSIN;

  dg::SignalPtr<dynamicgraph::Vector, int> velocitySIN;
  dg::SignalPtr<dynamicgraph::Vector, int> accelerationSIN;

  /* --- INTERMEDIATE OUTPUTS --- */
  dg::SignalTimeDependent<MatrixForce, int> handXworldSOUT;
  dg::SignalTimeDependent<MatrixForce, int> handVsensorSOUT;
  dg::SignalPtr<dynamicgraph::Vector, int> torsorDeadZoneSIN;

  dg::SignalTimeDependent<MatrixForce, int> sensorXhandSOUT;
  // dg::SignalTimeDependent<dynamicgraph::Matrix,int> inertiaSensorSOUT;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> momentumSOUT;
  dg::SignalPtr<dynamicgraph::Vector, int> momentumSIN;

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<dynamicgraph::Vector, int> torsorCompensatedSOUT;
  dg::SignalTimeDependent<dynamicgraph::Vector, int> torsorDeadZoneSOUT;

  typedef int sotDummyType;
  dg::SignalTimeDependent<sotDummyType, int> calibrationTrigerSOUT;

 public: /* --- COMMANDLINE --- */
  sotDummyType& calibrationTriger(sotDummyType& dummy, int time);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_SOTFORCECOMPENSATION_H__
