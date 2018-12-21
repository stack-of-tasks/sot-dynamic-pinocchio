/*
 * Copyright 2010-2018, CNRS
 * Florent Lamiraux
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_PINOCCHIO_DEVICE_HH
#define SOT_PINOCCHIO_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <urdf_parser/urdf_parser.h>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/math/quaternion.hpp"

namespace dynamicgraph {
  namespace sot {

    /// Specifies the nature of one joint control
    /// It is used for both the SoT side and the hardware side.
    enum ControlType
      {
	POSITION=0,
	VELOCITY=1,
	ACCELERATION=2,
	TORQUE=3
      };

    const std::string ControlType_s[] =
    {
      "POSITION", "VELOCITY", "ACCELERATION","TORQUE"
    };

    //@}

    
    struct JointSoTHWControlType
    {
      ControlType SoTcontrol;
      ControlType HWcontrol;
      int control_index;
      int pinocchio_index;
    };

    typedef std::map<std::string,JointSoTHWControlType>::iterator
    JointSHControlType_iterator;
    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class SOT_CORE_EXPORT PinocchioDevice
        :public Entity
    {
    public:
      static const std::string CLASS_NAME;
      virtual const std::string& getClassName(void) const {
        return CLASS_NAME;
      }

      enum ForceSignalSource
      {
        FORCE_SIGNAL_RLEG,
        FORCE_SIGNAL_LLEG,
        FORCE_SIGNAL_RARM,
        FORCE_SIGNAL_LARM
      };
      
    protected:

      /// \name Vectors related to the state.
      ///@{
      /// State vector of the robot (deprecated)
      dg::Vector state_;
      
      /// Position of the robot wrt pinocchio.
      Eigen::VectorXd position_;
      /// Velocity of the robot wrt pinocchio.
      Eigen::VectorXd velocity_;
      /// Acceleration vector of each actuator.
      dg::Vector acceleration_;
      
      /// Torque vector of each actuator.
      dg::Vector torque_;
      ///@}


      bool sanityCheck_;
      
      /// Specifies the control input by each element of the state vector.
      std::map<std::string,ControlType> sotControlType_;
      std::map<std::string,ControlType> hwControlType_;

      /// Maps of joint devices.
      std::map<std::string,JointSoTHWControlType> jointPinocchioDevices_;
      /// 
      bool withForceSignals[4];
      PeriodicCall periodicCallBefore_;
      PeriodicCall periodicCallAfter_;

    public:

      /* --- CONSTRUCTION --- */
      PinocchioDevice(const std::string& name);
      /* --- DESTRUCTION --- */
      virtual ~PinocchioDevice();

      virtual void setState(const dg::Vector& st);
      void setVelocitySize(const unsigned int& size);
      virtual void setVelocity(const dg::Vector & vel);

      /// Set control input type.
      virtual void setSoTControlType(const std::string &jointNames,
				     const std::string &sotCtrlType);
      virtual void setHWControlType(const std::string &jointNames,
				    const std::string &hwCtrlType);
      virtual void increment(const double & dt = 5e-2);
      /// Read directly the URDF model
      void setURDFModel(const std::string &aURDFModel);
      
      /// \name Sanity check parameterization
      /// \{
      void setSanityCheck   (const bool & enableCheck);
      /// \}

      /// \name Set index in vector (position, velocity, acceleration, control)
      /// \{
      void setControlPos(const std::string &jointName,
			 const unsigned & index);
      /// \}
    public: /* --- DISPLAY --- */
      virtual void display(std::ostream& os) const;
      SOT_CORE_EXPORT friend std::ostream&
      operator<<(std::ostream& os,const PinocchioDevice& r) {
        r.display(os); return os;
      }

    public: /* --- SIGNALS --- */

      /// Input signal handling the control vector
      /// This entity needs a control vector to be send to the hardware.
      /// The control vector can be position, velocity and effort.
      /// It depends on each of the actuator
      dynamicgraph::SignalPtr<dg::Vector,int> controlSIN;

      /// \name This part is specific to robot where a stabilizer is provided outside the
      /// SoT framework and needs input.
      /// @{ 
      /// Input signal handling the attitude of the freeflyer.
      dynamicgraph::SignalPtr<dg::Vector,int> attitudeSIN;
      /// Input signal handling the ZMP of the system 
      dynamicgraph::SignalPtr<dg::Vector,int> zmpSIN;
      ///@}
      
      /// \name PinocchioDevice current state.
      /// \{
      /// \brief Output integrated state from control.
      dynamicgraph::Signal<dg::Vector,int> stateSOUT;
      /// \brief Output integrated velocity from control
      dynamicgraph::Signal<dg::Vector,int> velocitySOUT;
      /// \brief Output attitude provided by the hardware
      /// Typically this can be provided by an external estimator
      /// such an integrated/hardware implemented EKF.
      dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
      /*! \brief The current state of the robot from the command viewpoint. */
      dynamicgraph::Signal<dg::Vector,int> motorcontrolSOUT;
      dynamicgraph::Signal<dg::Vector,int> previousControlSOUT;
      /*! \brief The ZMP reference send by the previous controller. */
      dynamicgraph::Signal<dg::Vector,int> ZMPPreviousControllerSOUT;
      /// \}

      /// \name Real robot current state
      /// This corresponds to the real encoders values and take into
      /// account the stabilization step. Therefore, this usually
      /// does *not* match the state control input signal.
      /// \{
      /// Motor positions
      dynamicgraph::Signal<dg::Vector, int> robotState_;
      /// Motor velocities
      dynamicgraph::Signal<dg::Vector, int> robotVelocity_;
      /// The force torque sensors
      dynamicgraph::Signal<dg::Vector,int>* forcesSOUT[4];
      /// Motor torques
      /// \todo why pseudo ?
      dynamicgraph::Signal<dg::Vector,int> pseudoTorqueSOUT;
      /// \}

    protected:
      void setControlType(const std::string &strCtrlType,
			  ControlType &aCtrlType);
      
      /// Compute roll pitch yaw angles of freeflyer joint.
      void integrateRollPitchYaw(dg::Vector& state, const dg::Vector& control,
                                 double dt);
      /// Store Position of free flyer joint
      MatrixHomogeneous ffPose_;
      /// Compute the new position, from the current control.
      ///
      /// When sanity checks are enabled, this checks that the control is within
      /// bounds. There are three cases, depending on what the control is:
      /// - position: checks that the position is within bounds,
      /// - velocity: checks that the velocity and the future position are
      ///             within bounds,
      /// - acceleration: checks that the acceleration, the future velocity and
      ///                 position are within bounds.
      ///                 \todo in order to check the acceleration, we need
      ///                 pinocchio and the contact forces in order to estimate
      ///                 the joint torques for the given acceleration.
      virtual void integrate( const double & dt );
    protected:
      /// Get freeflyer pose
      const MatrixHomogeneous& freeFlyerPose() const;
    public:
      virtual void setRoot( const dg::Matrix & root );


      virtual void setRoot( const MatrixHomogeneous & worldMwaist );

    private:
      // Intermediate variable to avoid dynamic allocation
      dg::Vector forceZero6;

      // Pinocchio Model of the robot
      se3::Model model_;

    public:
      const se3::Model & getModel()
      { return model_;}
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_PINOCCHIO_DEVICE_HH */




