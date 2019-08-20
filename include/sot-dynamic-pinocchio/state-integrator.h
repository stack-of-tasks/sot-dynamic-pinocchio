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
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
#include "pinocchio/multibody/model.hpp"
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

/// Specifies the nature of one joint control from SoT side
enum SoTControlType {
  VEL = 0,
  ACC = 1
};

const std::string SoTControlType_s[] = {
  "VELOCITY", "ACCELERATION"
};



/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSTATEINTEGRATOR_EXPORT StateIntegrator: public Entity 
{
 public:

  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const {return CLASS_NAME;}
  static const double TIMESTEP_DEFAULT;

  /// Set integration time.
  void timeStep(double ts) { timestep_ = ts;}

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

  /// Compute the new position, from the current control.
  /// When sanity checks are enabled, this checks that the control has no NAN value.
  /// There are two cases, depending on what the control is:
  /// - velocity: integrate once to obtain the future position 
  /// - acceleration: integrate two times to obtain the future position 
  virtual void integrate( const double & dt = 5e-2 );

  /// Read directly the URDF model
  void setURDFModel(const std::string &aURDFModel);

  /// \name Sanity check parameterization
  /// \{
  void setSanityCheck(const bool & enableCheck);
  /// \}

  /// \name Get the control type from an int (of the controlTypeSIN signal) as in the enum
  /// Check the types: velocity = 0 or acceleration = 1
  /// \{  
  int getControlType(const int &ctrlType, SoTControlType &aCtrlType);
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
  /// Input signal handling the type of the control vector 
  /// It can be velocity or acceleration.
  /// It depends on each of the actuator
  dg::SignalPtr<dg::Vector, int> controlTypeSIN;

  /// \name StateIntegrator current state.
  /// \{
  /// \brief Output integrated state from control.
  dg::Signal<dg::Vector, int> stateSOUT_;
  /// \brief Output integrated velocity from control.
  dg::Signal<dg::Vector, int> velocitySOUT_;
  /// \brief Output integrated freeFlyer position from control (odometry predictive system).
  dg::Signal<dg::Vector, int> freeFlyerPositionOdomSOUT_;
  /// \brief Output integrated freeFlyer velocity from control.
  dg::Signal<dg::Vector, int> freeFlyerVelocitySOUT_;
  /// \}

  /// \brief Provides the itegrated control information.
  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);
  ///@}

 protected:

  /// Integrate the freeflyer state (to obtain position or velocity).
  /// Compute roll pitch yaw angles
  /// Publish the result on the dedicated signal.
  void integrateRollPitchYaw(dg::Vector& state, dg::Signal<dg::Vector, int>& signal, const dg::Vector& control, double dt);

  /// Store Position of free flyer joint as MatrixHomogeneous
  MatrixHomogeneous freeFlyerPose_;



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

 public:

  const pinocchio::Model & getModel()
  { 
    return model_;
  }

};
} // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_STATEINTEGRATOR_HH */