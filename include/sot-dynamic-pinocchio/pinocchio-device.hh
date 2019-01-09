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

/// URDF DOM
#include <urdf_parser/urdf_parser.h>

/// YAML CPP
#include <yaml-cpp/yaml.h>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
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

namespace dgsot=dynamicgraph::sot;
namespace dg=dynamicgraph;

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

    /// \brief Store information on each joint.
    struct JointSoTHWControlType
    {
      /// Defines the control from the Stack-of-Tasks side (for instance, velocity)
      ControlType SoTcontrol;
      /// Defines the control from the hardware side.
      ControlType HWcontrol;
      /// Position of the joint in the control vector.
      int control_index;
      /// Position of the joint in the pinocchio/URDF index.
      int pinocchio_index;

      /// Various indexes for the sensor signals.
      /// This may vary if some joints does not support this feature.
      ///@{
      /// Position of the joint in the temperature vector
      int temperature_index;

      /// Position of the joint in the velocity vector
      int velocity_index;
      
      /// Position of the joint in the current vector
      int current_index;

      /// Position of the joint in the torque vector
      int torque_index;
      
      /// Position of the joint in the joint-angles vector
      int joint_angle_index;

      /// Position of the joint in the motor-angles vector
      int motor_angle_index;

      ///@}
      JointSoTHWControlType();
    };

    struct IMUSOUT
    {
      std::string imu_sensor_name;
      dg::Signal<MatrixRotation, int> attitudeSOUT;
      dg::Signal<dg::Vector,int> accelerometerSOUT;
      dg::Signal<dg::Vector,int> gyrometerSOUT;
      IMUSOUT(const std::string &limu_sensor_name,
	      const std::string &device_name):
	imu_sensor_name(limu_sensor_name)
	,attitudeSOUT("PinocchioDevice("+device_name+
		     ")::output(vector6)::"+imu_sensor_name+"_attitudeSOUT")
	,accelerometerSOUT("PinocchioDevice("+device_name+
			   ")::output(vector3)::"+imu_sensor_name+"_accelerometerSOUT")
	,gyrometerSOUT("PinocchioDevice("+device_name+
		       ")::output(vector3)::"+imu_sensor_name+"_gyrometerSOUT")
      {}
    };
    typedef std::map<std::string,JointSoTHWControlType>::iterator
    JointSHWControlType_iterator;
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
      static const double TIMESTEP_DEFAULT;

      /// Maps of joint devices.
      std::map<std::string,dgsot::JointSoTHWControlType> jointPinocchioDevices_;

    protected:
      /// \brief Current integration step.
      double timestep_;

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

      /// 
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
      dg::SignalPtr<dg::Vector,int> controlSIN;

      
      /// \name PinocchioDevice current state.
      /// \{
      /// \brief Output integrated state from control.
      dg::Signal<dg::Vector,int> stateSOUT_;
      /// \brief Output integrated velocity from control
      dg::Signal<dg::Vector,int> velocitySOUT_;
      /// \brief Output attitude provided by the hardware
      /*! \brief The current state of the robot from the command viewpoint. */
      dg::Signal<dg::Vector,int> motorcontrolSOUT_;
      dg::Signal<dg::Vector,int> previousControlSOUT_;
      /// \}

      /// \name Real robot current state
      /// This corresponds to the real encoders values and take into
      /// account the stabilization step. Therefore, this usually
      /// does *not* match the state control input signal.
      /// \{
      /// Motor positions
      dg::Signal<dg::Vector, int> robotState_;
      /// Motor velocities
      dg::Signal<dg::Vector, int> robotVelocity_;
      /// The force torque sensors
      std::vector<dg::Signal<dg::Vector,int>*> forcesSOUT_;
      /// The imu sensors
      std::vector<IMUSOUT *> imuSOUT_;
      /// Motor or pseudo torques (estimated or build from PID)
      dg::Signal<dg::Vector,int> * pseudoTorqueSOUT_;
      /// Temperature signal
      dg::Signal<dg::Vector,int> * temperatureSOUT_;
      /// Current signal
      dg::Signal<dg::Vector,int> * currentsSOUT_;
      /// Motor angles signal
      dg::Signal<dg::Vector, int> * motor_anglesSOUT_;
      /// Joint angles signal
      dg::Signal<dg::Vector, int> * joint_anglesSOUT_;
      /// P gains signal
      dg::Signal<dg::Vector,int> * p_gainsSOUT_;
      /// D gains signal
      dg::Signal<dg::Vector,int> * d_gainsSOUT_;
      /// \}

      /// Parse a YAML string for configuration.
      int ParseYAMLString(const std::string &aYamlString);

      /// \name Robot Side
      ///@{

      /// \brief Allows the robot to fill in the internal variables of the device
      /// to publish data on the signals.
      void setSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
      void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
      void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
      void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);
      
      /// \brief Provides to the robot the control information.
      void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);
      ///@}
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

      /// \name YAML related methods
      /// @{
      /// Parse YAML for mapping between hardware and sot
      /// starting from a YAML-cpp node.
      int ParseYAMLMapHardware(YAML::Node &map_hs_control);

      /// Parse YAML for sensors from a YAML-cpp node.
      int ParseYAMLSensors(YAML::Node &map_sensors);

      /// Parse YAML for joint sencors from YAML-cpp node.
      int ParseYAMLJointSensor(std::string & joint_name,
			       YAML::Node &aJointSensors);
      /// @}

      /// \name Signals related methods
      ///@{
      /// \brief Creates a signal called PinocchioDevice(DeviceName)::output(vector6)::force_sensor_name
      void CreateAForceSignal(const std::string & force_sensor_name);
      /// \brief Creates a signal called PinocchioDevice(DeviceName)::output(vector6)::imu_sensor_name
      void CreateAnImuSignal(const std::string & imu_sensor_name);

      /// \brief Creates signals based on the joints information parsed by the YAML string.
      int UpdateSignals();
      
      ///@}
      /// Get freeflyer pose
      const MatrixHomogeneous& freeFlyerPose() const;

      /// Protected methods for internal variables filling
      void setSensorsForce(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
      void setSensorsIMU(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
      void setSensorsEncoders(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
      void setSensorsVelocities(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
      void setSensorsTorquesCurrents(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
      
  void setSensorsGains(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);

    public:
      virtual void setRoot( const dg::Matrix & root );
      virtual void setRoot( const MatrixHomogeneous & worldMwaist );

    private:
      
      // Intermediate variable to avoid dynamic allocation
      dg::Vector forceZero6;

      // Pinocchio Model of the robot
      se3::Model model_;

      // Debug mode
      int debug_mode_;

      // Intermediate index when parsing YAML file.
      int temperature_index_,velocity_index_,
	current_index_,torque_index_,
	joint_angle_index_,
	motor_angle_index_
	;
    public:
      const se3::Model & getModel()
      { return model_;}
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_PINOCCHIO_DEVICE_HH */




