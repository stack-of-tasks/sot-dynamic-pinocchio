/*
 * Copyright 2010-2018, CNRS
 * Florent Lamiraux
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_STATEINTEGRATOR_HH
#define SOT_STATEINTEGRATOR_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <vector>
#include <math.h>
/* SOT */
/// dg
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
/// sot-core
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"
#include <sot/core/abstract-sot-external-interface.hh>
/// pinocchio
#include "pinocchio/multibody/model.hpp"
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/math/quaternion.hpp"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (state_integrator_EXPORTS)
#    define SOTSTATEINTEGRATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTSTATEINTEGRATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSTATEINTEGRATOR_EXPORT
#endif


namespace dgsot = dynamicgraph::sot;
namespace dg = dynamicgraph;

namespace dynamicgraph {
namespace sot {

typedef Eigen::Matrix<std::string, Eigen::Dynamic, 1> StringVector;

/// Specifies the nature of one joint control from SoT side
enum SoTControlType {
  qVEL = 0,
  qACC = 1,
  ffVEL = 2,
  ffACC = 3
};

const std::string SoTControlType_s[] = {
  "qVEL", "qACC", "ffVEL", "ffACC"
};



/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSTATEINTEGRATOR_EXPORT StateIntegrator: public Entity {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const {return CLASS_NAME;}
  static const double TIMESTEP_DEFAULT;

  /// Set integration time.
  void init(const double& step);

 protected:
  /// \brief Current integration step.
  double timestep_;

  /// \name Vectors related to the state.
  ///@{
  /// Position of the robot wrt pinocchio.
  Eigen::VectorXd position_;
  /// Velocity of the robot wrt pinocchio.
  Eigen::VectorXd velocity_;
  /// Acceleration vector of each actuator.
  dg::Vector acceleration_;

  /// Type of the control vector
  /// It can be velocity or acceleration for the actuators.
  std::vector<SoTControlType> controlTypeVector_;
  /// Type of the control for the Freeflyer (velocity/acceleration).
  SoTControlType controlTypeFF_;

  /// Store Position of free flyer joint
  Eigen::VectorXd ffPose_;
  /// Store Velocity of free flyer joint
  Eigen::VectorXd ffVel_;
  ///@}

  bool sanityCheck_;

 public:

  /* --- CONSTRUCTION --- */
  StateIntegrator(const std::string& name);
  /* --- DESTRUCTION --- */
  virtual ~StateIntegrator();

  virtual void setState(const dg::Vector& st);
  void setVelocitySize(const unsigned int& size);
  virtual void setVelocity(const dg::Vector & vel);

  virtual void setStateFreeflyer( const dg::Vector& st);
  virtual void setVelocityFreeflyer(const dg::Vector & vel);

  /// Read directly the URDF model
  void setURDFModel(const std::string &aURDFModel);

  /// \name Sanity check parameterization
  /// \{
  void setSanityCheck(const bool & enableCheck);
  /// \}

  /// \name Set the control types of the controlled joints
  /// Allowed types (string): qVEL | qACC
  void setControlType(const StringVector& controlTypeVector);
  /// \name Set the control types of the controlled freeflyer
  /// Allowed types (string): ffVEL | ffACC
  void setControlTypeFreeFlyer(const std::string& controlTypeFF);

  /// \name Set the control type of a specific joint 
  /// Allowed types (int): qVEL:0 | qACC:1 
  void setControlTypeJointInt(const int& jointNumber, const int& intType);
  /// \name Set the control types of the controlled joints
  /// Allowed types (int): qVEL:0 | qACC:1 
  void setControlTypeInt(const Vector& controlTypeVector);

  /// \name Get the control type from a string (of the controlTypeVector) as in the enum
  /// Check the types: qVEL | qACC | ffVEL | ffACC
  /// \{
  int getControlType(const std::string &strCtrlType, SoTControlType &aCtrlType);
  /// \}

 public: /* --- DISPLAY --- */
  virtual void display(std::ostream& os) const;
  SOT_CORE_EXPORT friend std::ostream& operator<<(std::ostream& os, const StateIntegrator& r) {
    r.display(os); return os;
  }

 public: /* --- SIGNALS --- */

  /// Input signal handling the control vector
  /// This entity needs a control vector to be send to the device.
  dg::SignalPtr<dg::Vector, int> controlSIN;
  /// Input signal handling the control vector of the freeflyer
  dg::SignalPtr<dg::Vector, int> freeFlyerSIN;

  /// \name StateIntegrator current state.
  /// \{
  /// \brief Output integrated state from control.
  dg::SignalTimeDependent<dg::Vector, int> stateSOUT_;
  /// \brief Output integrated velocity from control.
  dg::SignalTimeDependent<dg::Vector, int> velocitySOUT_;
  /// \brief Output integrated freeFlyer position from control (odometry predictive system) with euler angles.
  dg::SignalTimeDependent<dg::Vector, int> freeFlyerPositionEulerSOUT_;
  /// \brief Output integrated freeFlyer position from control (odometry predictive system) with quaternions.
  dg::SignalTimeDependent<dg::Vector, int> freeFlyerPositionQuatSOUT_;
  /// \brief Output integrated freeFlyer velocity from control.
  dg::SignalTimeDependent<dg::Vector, int> freeFlyerVelocitySOUT_;
  /// \}
  ///@}

 protected:

  /// Integrate the freeflyer state (to obtain position).
  /// Compute roll pitch yaw angles
  void integrateRollPitchYaw(dg::Vector& state, const dg::Vector& control, double dt);
  // Computes Euler angles in good range : [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]
  void rotationMatrixToEuler(Eigen::Matrix3d& rotationMatrix, Eigen::Vector3d& rollPitchYaw);

  /// Store Position of free flyer joint as MatrixHomogeneous
  MatrixHomogeneous freeFlyerPose_;

  /// Compute the new position, from the current control.
  /// When sanity checks are enabled, this checks that the control has no NAN value.
  /// There are two cases, depending on what the control is:
  /// - velocity: integrate once to obtain the future position
  /// - acceleration: integrate two times to obtain the future position
  virtual void integrateControl(int t, const double & dt = 5e-2);
  /// Compute the new freeflyer position, from the current control.
  /// When sanity checks are enabled, this checks that the control has no NAN value.
  /// There are two cases, depending on what the control is:
  /// - velocity: integrate once to obtain the future position
  /// - acceleration: integrate two times to obtain the future position
  virtual void integrateFreeFlyer(int t, const double & dt = 5e-2);

  /// \brief Provides the itegrated control information in position (callback signal stateSOUT_).
  dg::Vector& getPosition(dg::Vector &controlOut, const int& t);
  /// \brief Provides the itegrated control information in velocity (callback signal velocitySOUT_).
  dg::Vector& getVelocity(dg::Vector &controlOut, const int& t);
  /// \brief Provides the itegrated control information of the freeflyer in position with euler angles
  ///  (callback signal freeFlyerPositionEulerSOUT_).
  dg::Vector& getFreeFlyerPositionEuler(dg::Vector &ffPose, const int& t);
  /// \brief Provides the itegrated control information of the freeflyer in position with quaternions
  ///  (callback signal freeFlyerPositionQuatSOUT_).
  dg::Vector& getFreeFlyerPositionQuat(dg::Vector &ffPose, const int& t);
  /// \brief Provides the itegrated control information of the freeflyer in velocity (callback signal freeFlyerVelocitySOUT_).
  dg::Vector& getFreeFlyerVelocity(dg::Vector &ffVel, const int& t);

 public:
  /// Get freeflyer pose
  const MatrixHomogeneous& freeFlyerPose();
  virtual void setRoot( const dg::Matrix & root );
  virtual void setRoot( const MatrixHomogeneous & worldMwaist );

 private:

  // Pinocchio Model of the robot
  pinocchio::Model model_;
  // Debug mode
  int debug_mode_;
  // Last integration iteration
  int last_integration_;
  // Last integration iteration for freeflyer
  int last_integration_FF_;

 public:

  const pinocchio::Model & getModel() {
    return model_;
  }

};
} // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_STATEINTEGRATOR_HH */