/*
 * Copyright 2019,  CNRS
 * Author: Olivier Stasse
 *
 * Please check LICENSE.txt for licensing
 * 
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot-dynamic-pinocchio/pinocchio-device.hh"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-geometry.hh>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string PinocchioDevice::CLASS_NAME = "PinocchioDevice";


JointSoTHWControlType::JointSoTHWControlType():
  control_index(-1)
  ,pinocchio_index(-1)
  ,temperature_index(-1)
  ,velocity_index(-1)
  ,current_index(-1)
  ,torque_index(-1)
  ,joint_angle_index(-1)
  ,motor_angle_index(-1)
{
}
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


const MatrixHomogeneous& PinocchioDevice::freeFlyerPose() const
{
  return ffPose_;
}

PinocchioDevice::
~PinocchioDevice( )
{
  for( unsigned int i=0; i<forcesSOUT_.size(); ++i ) {
    delete forcesSOUT_[i];
  }

  for( unsigned int i=0; i<imuSOUT_.size(); ++i ) {
    delete imuSOUT_[i];
  }

}

PinocchioDevice::
PinocchioDevice( const std::string& n )
  :Entity(n)
  ,position_(6)
  ,sanityCheck_(true)
  ,controlSIN( NULL,"PinocchioDevice("+n+")::input(double)::control" )   
  ,stateSOUT_( "PinocchioDevice("+n+")::output(vector)::state" )   
  ,velocitySOUT_( "PinocchioDevice("+n+")::output(vector)::velocity"  )
  ,motorcontrolSOUT_   ( "PinocchioDevice("+n+")::output(vector)::motorcontrol" )
  ,previousControlSOUT_( "PinocchioDevice("+n+")::output(vector)::previousControl" )
  ,robotState_     ("PinocchioDevice("+n+")::output(vector)::robotState")
  ,robotVelocity_  ("PinocchioDevice("+n+")::output(vector)::robotVelocity")
  ,ffPose_()
  ,forceZero6 (6)
  ,debug_mode_(5)
  ,temperature_index_(0)
  ,velocity_index_(0)
  ,current_index_(0)
  ,torque_index_(0)
  ,joint_angles_index_(0)
  ,motor_angles_index_(0)
   
{
  forceZero6.fill (0);

  signalRegistration( controlSIN
		      << stateSOUT_
		      << robotState_
		      << robotVelocity_
                      << velocitySOUT_
		      << previousControlSOUT_
		      << motorcontrolSOUT_);
  position_.fill(.0); stateSOUT_.setConstant( position_ );

  velocity_.resize(position_.size()); velocity_.setZero();
  velocitySOUT_.setConstant( velocity_ );

  /* --- Commands --- */
  {
    std::string docstring;
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set",
               new command::Setter<PinocchioDevice, Vector>
               (*this, &PinocchioDevice::setState, docstring));

    docstring =
        "\n"
        "    Set velocity vector value\n"
        "\n";
    addCommand("setVelocity",
               new command::Setter<PinocchioDevice, Vector>
               (*this, &PinocchioDevice::setVelocity, docstring));

    void(PinocchioDevice::*setRootPtr)(const Matrix&) = &PinocchioDevice::setRoot;
    docstring
        = command::docCommandVoid1("Set the root position.",
                                   "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this,setRootPtr,
                                         docstring));



    /* SET of SoT control input type per joint */
    docstring =
        "\n"
        "    Set the type of control input per joint on the SoT side \n"
        "    which can be  \n"
        "    torque, acceleration, velocity, or position\n"
        "\n";

    addCommand("setSoTControlType",
	       command::makeCommandVoid2(*this,&PinocchioDevice::setSoTControlType,
                 command::docCommandVoid2 ("Set SoT control input type per joint",
					   "Joint name",
					   "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));


    /* SET of HW control input type per joint */
    docstring =
        "\n"
        "    Set the type of control input per joint which can be  \n"
        "    torque, acceleration, velocity, or position\n"
        "\n";

    addCommand("setHWControlType",
	       command::makeCommandVoid2(*this,&PinocchioDevice::setHWControlType,
                 command::docCommandVoid2 ("Set HW control input type per joint",
					   "Joint name",
					   "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));
    
    docstring =
        "\n"
        "    Enable/Disable sanity checks\n"
        "\n";
    addCommand("setSanityCheck",
               new command::Setter<PinocchioDevice, bool>
               (*this, &PinocchioDevice::setSanityCheck, docstring));




    // Handle commands and signals called in a synchronous way.
    periodicCallBefore_.addSpecificCommands(*this, commandMap, "before.");
    periodicCallAfter_.addSpecificCommands(*this, commandMap, "after.");

  }
}

void PinocchioDevice::
setState( const Vector& st )
{
  if (sanityCheck_) {
  }
  position_ = st;
  stateSOUT_ .setConstant( position_ );
  motorcontrolSOUT_ .setConstant( position_ );
}

void PinocchioDevice::
setVelocity( const Vector& vel )
{
  velocity_ = vel;
  velocitySOUT_ .setConstant( velocity_ );
}

void PinocchioDevice::
setRoot( const Matrix & root )
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void PinocchioDevice::
setRoot( const MatrixHomogeneous & worldMwaist )
{
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2,1,0)).reverse();
  Vector q = position_;
  q = worldMwaist.translation(); // abusive ... but working.
  for( unsigned int i=0;i<3;++i ) q(i+3) = r(i);
}


void PinocchioDevice::
setSanityCheck(const bool & enableCheck)
{
  sanityCheck_ = enableCheck;
}

void PinocchioDevice::
setControlType(const std::string &strCtrlType,
	       ControlType &aCtrlType)
{
  for(int j=0;j<4;j++)
    {
      if (strCtrlType==ControlType_s[j])
	aCtrlType = (ControlType)j;
    }
}

void PinocchioDevice::
setSoTControlType(const std::string &jointNames,
		  const std::string &strCtrlType)
{
  setControlType(strCtrlType,jointPinocchioDevices_[jointNames].SoTcontrol);
}

void PinocchioDevice::
setHWControlType(const std::string &jointNames,
		 const std::string &strCtrlType)
{
  setControlType(strCtrlType,jointPinocchioDevices_[jointNames].HWcontrol);
}

void PinocchioDevice::
setControlPos(const std::string &jointName,
	       const unsigned & index)
{
  jointPinocchioDevices_[jointName].control_index = index;
}


void PinocchioDevice::
setURDFModel(const std::string &aURDFModel)
{
  se3::urdf::buildModelFromXML(aURDFModel,model_,false);

  /// Build the map between pinocchio and the alphabetical order.
  for(unsigned int i=0;i<model_.names.size();i++)
    jointPinocchioDevices_[model_.names[i]].pinocchio_index=i;

  // Initialize pinocchio vector.
  position_.resize(model_.nq);
  velocity_.resize(model_.nv);
  acceleration_.resize(model_.nv);
}

void PinocchioDevice::
increment( const double & dt )
{
  int time = stateSOUT_.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallBefore_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }


  /* Force the recomputation of the control. */
  controlSIN( time );
  sotDEBUG(25) << "u" <<time<<" = " << controlSIN.accessCopy() << endl;

  integrate(dt);
  
  sotDEBUG(25) << "q" << time << " = " << position_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT_ .setConstant( position_ ); stateSOUT_.setTime( time+1 );


  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallAfter_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }


  // Others signals.
  motorcontrolSOUT_ .setConstant( position_ );
}


void PinocchioDevice::integrate( const double & dt )
{
  const Vector & controlIN = controlSIN.accessCopy();

  if (sanityCheck_ && controlIN.hasNaN())
  {
    dgRTLOG () << "PinocchioDevice::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }

  bool shouldIntegrateVelocity=false;
  
  JointSHWControlType_iterator it_control_type;
  for(it_control_type  = jointPinocchioDevices_.begin();
      it_control_type != jointPinocchioDevices_.end();
      it_control_type++)
    {

      int lctl_index = it_control_type->second.control_index;
      int pino_index = it_control_type->second.pinocchio_index;

      if (pino_index!=-1)
	{
	  int lpos_index = model_.joints[pino_index].idx_q();
	  int lvel_index = model_.joints[pino_index].idx_v();
	  if ((lpos_index==-1) || (lvel_index==-1))
	    break;
	  
	  double lvelocityLimit = model_.velocityLimit[lvel_index];
	  	  
	  // Integration.
	  if (it_control_type->second.SoTcontrol==ACCELERATION)
	    {
	      acceleration_[lvel_index] = controlIN[lctl_index];
	      
	      velocity_[lvel_index] = velocity_[lvel_index] +
		acceleration_[lvel_index]*dt;
	      if (velocity_[lvel_index]<-lvelocityLimit)
		velocity_[lvel_index] = -lvelocityLimit;
	      else if (velocity_[lvel_index]>lvelocityLimit)
		velocity_[lvel_index] = lvelocityLimit;
	      
	      shouldIntegrateVelocity=true;
	    }
	  else if (it_control_type->second.SoTcontrol==VELOCITY)
	    {
	      acceleration_[lvel_index]=0;
	      velocity_[lvel_index] = controlIN[lctl_index];
	      if (velocity_[lvel_index]<-lvelocityLimit)
		velocity_[lvel_index] = -lvelocityLimit;
	      else if (velocity_[lvel_index]>lvelocityLimit)
		velocity_[lvel_index] = lvelocityLimit;
	      
	      shouldIntegrateVelocity=true;
	    }
	  else if (it_control_type->second.SoTcontrol==POSITION)
	    {
	      acceleration_[lvel_index]=0;
	      velocity_[lvel_index] = 0;
	      position_[lpos_index] = controlIN[lctl_index];
	      if (position_[lpos_index]<model_.lowerPositionLimit[lpos_index])
		position_[lpos_index]=model_.lowerPositionLimit[lpos_index];
	      else if (position_[lpos_index]>model_.upperPositionLimit[lpos_index])
		position_[lpos_index]=model_.upperPositionLimit[lpos_index];
	      
	    }
	}
    }
  
  // Velocity integration
  if (shouldIntegrateVelocity)
    {
      // Pinocchio SE3 integration.
      position_ = se3::integrate(model_,position_,velocity_);
      
      /// Checking values.
      for(it_control_type  = jointPinocchioDevices_.begin();
	  it_control_type != jointPinocchioDevices_.end();
	  it_control_type++)
	{
	  int pino_index = it_control_type->second.pinocchio_index;
	  if (pino_index!=-1)
	    {
	      int lpos_index = model_.joints[pino_index].idx_q();
	      if (lpos_index==-1)
		break;
	      
	      if (position_[lpos_index]<model_.lowerPositionLimit[lpos_index])
		position_[lpos_index]=model_.lowerPositionLimit[lpos_index];
	      else if (position_[lpos_index]>model_.upperPositionLimit[lpos_index])
		position_[lpos_index]=model_.upperPositionLimit[lpos_index];
	    }
	}
      
      stateSOUT_ .setConstant( position_ );
      velocitySOUT_ .setConstant( velocity_ );
      
    }
}


int PinocchioDevice::
ParseYAMLString(const std::string & aYamlString)
{
  YAML::Node map_global = YAML::Load(aYamlString);

  YAML::Node map_sot_controller = map_global["sot_controller"];

  for (YAML::const_iterator it=map_sot_controller.begin();
       it!=map_sot_controller.end();
       it++)
    {
      if (debug_mode_>1)
	{
	  std::cout << "key:" << it->first.as<string>() << std::endl;
	}
      std::string name_elt_of_sot_controller = it->first.as<string>();

      YAML::Node elt_of_sot_controller = it->second;
      
      if (name_elt_of_sot_controller=="map_hardware_sot_control")
	{
	  int r=ParseYAMLMapHardware(elt_of_sot_controller);
	  if (r<0) return r;
	}
      else if (name_elt_of_sot_controller=="sensors")
	{ int r=ParseYAMLSensors(elt_of_sot_controller);
	  if (r<0) return r;
	}
    }
  return 0;
}

int PinocchioDevice::
ParseYAMLJointSensor(std::string & joint_name,
		     YAML::Node &aJointSensor)
{
  JointSoTHWControlType & aJointSoTHWControlType =
    jointPinocchioDevices_[joint_name];

  if (debug_mode_>1)
    std::cout << "JointSensor for " << joint_name << std::endl;
  
  for (std::size_t i=0;i<aJointSensor.size();i++)
    {
      std::string  aSensor = aJointSensor[i].as<string>();
      if (debug_mode_>1)
	std::cout << "Found " << aSensor<< std::endl;

      if (aSensor=="temperature")
	{
	  aJointSoTHWControlType.temperature_index = temperature_index_;
	  temperature_index_++;
	}
      else if (aSensor=="velocity")
	{
	  aJointSoTHWControlType.velocity_index = velocity_index_;
	  velocity_index_++;
	}            
      else if (aSensor=="current")
	{
	  aJointSoTHWControlType.current_index = current_index_;
	  current_index_++;
	}
      else if (aSensor=="torque")
	{
	  aJointSoTHWControlType.torque_index = torque_index_;
	  torque_index_++;
	}
      else if (aSensor=="joint_angles")
	{
	  aJointSoTHWControlType.joint_angle_index = joint_angles_index_;
	  joint_angles_index_++;
	}
      else if (aSensor=="motor_angles")
	{
	  aJointSoTHWControlType.motor_angle_index = motor_angles_index_;
	  motor_angles_index_++;
	}
 
    }
  return 0;
}


int PinocchioDevice::
ParseYAMLMapHardware(YAML::Node & map_hs_control)
{
  if (debug_mode_>1)
    {
      std::cout << "map_hs_control.size(): "
		<< map_hs_control.size() << std::endl;
      std::cout << map_hs_control << std::endl;
    }
  unsigned int i=0;
  for (YAML::const_iterator it=map_hs_control.begin();
       it!=map_hs_control.end();
       it++)
    {
      if (debug_mode_>1)
	{
	  std::cout << i << " " << std::endl;
	  std::cout << "key:" << it->first.as<string>() << std::endl;
	}
      std::string jointName = it->first.as<string>();
      
      YAML::Node aNode = it->second;
      if (debug_mode_>1)
	std::cout << "Type of value: " << aNode.Type() << std::endl;
      
      for (YAML::const_iterator it2=aNode.begin();
	   it2!=aNode.end();
	   it2++)
	
	{
	  std::string aKey = it2->first.as<string>();
	  if (debug_mode_>1)
	    std::cout << "-- key:" << aKey << std::endl;

	  if (aKey=="hw")
	    {
	      std::string value = it2->second.as<string>();
	      if (debug_mode_>1)
		std::cout << "-- Value: " << value << std::endl;
	      setHWControlType(jointName,value);
	    }
	  else if (aKey=="sot")
	    {
	      std::string value = it2->second.as<string>();
	      if (debug_mode_>1)
		std::cout << "-- Value: " << value << std::endl;
	      setSoTControlType(jointName,value);
	    }
	  else if (aKey=="controlPos")
	    {
	      unsigned int index= it2->second.as<int>();
	      if (debug_mode_>1)
		std::cout << "-- index: " << index << std::endl;
	      setControlPos(jointName,index);
	    }
	  else if (aKey=="sensors")
	    {
	      YAML::Node aNode = it2->second;
	      if (debug_mode_>1)
		std::cout << "-- type: " << aNode.Type() << std::endl;
	      ParseYAMLJointSensor(jointName,aNode);
	    }

	}
      i++;
    }
  return 0;
}

/* Sensor signals */
int PinocchioDevice::
ParseYAMLSensors(YAML::Node &map_sensors)
{
  if (map_sensors.IsNull())
    {
      std::cerr << "PinocchioDevice::ParseYAMLString: No sensor detected in YamlString "  << std::endl;
      return -1;
    }

    for (YAML::const_iterator it=map_sensors.begin();
       it!=map_sensors.end();
       it++)
    {
      if (debug_mode_>1)
	{
	  std::cout << "sensor_type:" << it->first.as<string>() << std::endl;
	}
      std::string sensor_type = it->first.as<string>();
      
      YAML::Node aNode = it->second;
      if (debug_mode_>1)
	std::cout << "Type of value: " << aNode.Type() << std::endl;

      
      // Iterates over types of node.
      for (YAML::const_iterator it2=aNode.begin();
	   it2!=aNode.end();
	   it2++)
	
	{
	  std::string sensor_name = it2->first.as<string>();
	  if (debug_mode_>1)
	    std::cout << "-- sensor_name:" << sensor_name << std::endl;

	  if (sensor_type=="force_torque")
	    {
	      std::string force_sensor_name = it2->second.as<string>();
	      if (debug_mode_>1)
		std::cout << "-- force_sensor_name: " << force_sensor_name << std::endl;
	      CreateAForceSignal(force_sensor_name);
	    }
	  else if (sensor_type=="imu")
	    {
	      std::string imu_sensor_name = it2->second.as<string>();
	      if (debug_mode_>1)
		std::cout << "-- Value: " << imu_sensor_name << std::endl;
	      CreateAnImuSignal(imu_sensor_name);
	    }
	  else 
	    {
	      std::cerr << "The sensor type " << sensor_type
			<< " is not recognized" << std::endl;
	    }
	}
    }
    return 0;
}


void PinocchioDevice::CreateAForceSignal(const std::string & force_sensor_name)
{
  dynamicgraph::Signal<dg::Vector, int> * aForceSOUT_;
  /* --- SIGNALS --- */
  aForceSOUT_ =
    new Signal<Vector, int>("PinocchioDevice("+getName()+")::output(vector6)::" +
			    force_sensor_name);
  forcesSOUT_.push_back(aForceSOUT_);
  signalRegistration(*aForceSOUT_);
}

void PinocchioDevice::CreateAnImuSignal(const std::string &imu_sensor_name)
{
  dynamicgraph::Signal<dg::Vector, int> * anImuSOUT_;
  /* --- SIGNALS --- */
  anImuSOUT_ =
    new Signal<Vector, int>("PinocchioDevice("+getName()+")::output(vector6)::" +
			    imu_sensor_name);
  imuSOUT_.push_back(anImuSOUT_);
  signalRegistration(*anImuSOUT_);
}



int PinocchioDevice::
UpdateSignals()
{
  pseudoTorqueSOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::ptorque" );
  
  currentsSOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::currents" );
  
  temperatureSOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::temperatures");

  motor_anglesSOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::motor_angles");

  joint_anglesSOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::joint_angles");

  velocitySOUT_ = new Signal<Vector, int>
    ("PinocchioDevice("+getName()+")::output(vector)::joint_velocities");
}



/* --- DISPLAY ------------------------------------------------------------ */

void PinocchioDevice::display ( std::ostream& os ) const
{os <<name<<": "<<position_<<endl; }

/* Helpers for the controller */
void PinocchioDevice::setSensorsForce(map<string,dgsot::SensorValues> &SensorsIn,
				      int t)
{
  sotDEBUGIN(15);
  map<string,dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
  {

    // Implements force recollection.
    const vector<double>& forcesIn = it->second.getValues();
    for(int i=0;i<forcesSOUT_.size();++i)
    {
      sotDEBUG(15) << "Force sensor " << i << std::endl;
      for(int j=0;j<6;++j)
	{
	  dgforces_(j) = forcesIn[i*6+j];
	  sotDEBUG(15) << "Force value " << j << ":" << dgforces_(j) << std::endl;
	}
      forcesSOUT_[i]->setConstant(dgforces_);
      forcesSOUT_[i]->setTime (t);
    }
  }
  sotDEBUGIN(15);
}

void PinocchioDevice::setSensorsIMU(map<string,dgsot::SensorValues> &SensorsIn,
				    int t)
{
  map<string,dgsot::SensorValues>::iterator it;

  //TODO: Confirm if this can be made quaternion
  for(int i=0;i<imuSOUT_.size();++i)
    {
      it = SensorsIn.find("attitude");
      if (it!=SensorsIn.end())
	{
	  const vector<double>& attitude = it->second.getValues ();
	  for (unsigned int i = 0; i < 3; ++i)
	    for (unsigned int j = 0; j < 3; ++j)
	      pose (i, j) = attitude [i * 3 + j];
	  imuSOUT_[i].attitudeSOUT_.setConstant (pose);
	  imuSOUT_[i].attitudeSOUT_.setTime (t);
	}

      it = SensorsIn.find("accelerometer_0");
      if (it!=SensorsIn.end())
	{
	  const vector<double>& accelerometer =
	    SensorsIn ["accelerometer_0"].getValues ();
	  for (std::size_t i=0; i<3; ++i)
	    accelerometer_ (i) = accelerometer [i];
	  imuSOUT_[i].accelerometerSOUT_.setConstant (accelerometer_);
	  imuSOUT_[i].accelerometerSOUT_.setTime (t);
	}
      
      it = SensorsIn.find("gyrometer_0");
      if (it!=SensorsIn.end())
	{
	  const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
	  for (std::size_t i=0; i<3; ++i)
	    gyrometer_ (i) = gyrometer [i];
	  imuSOUT_[i].gyrometerSOUT_.setConstant (gyrometer_);
	  imuSOUT_[i].gyrometerSOUT_.setTime (t);
	}
    }
  }
}

void PinocchioDevice::
setSensorsEncoders(map<string,dgsot::SensorValues> &SensorsIn,
		   int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  
  it = SensorsIn.find("motor-angles");
  if (it!=SensorsIn.end())
  {
    const vector<double>& anglesIn = it->second.getValues();
    dgRobotState_.resize (anglesIn.size () + 6);
    motor_angles_.resize(anglesIn.size ());
    for (unsigned i = 0; i < 6; ++i)
      dgRobotState_ (i) = 0.;
    for (unsigned i = 0; i < anglesIn.size(); ++i)
      {
	dgRobotState_ (i + 6) = anglesIn[i];
	motor_angles_(i)= anglesIn[i];
      }
    robotState_.setConstant(dgRobotState_);
    robotState_.setTime(t);
    motor_anglesSOUT_.setConstant(motor_angles_);
    motor_anglesSOUT_.setTime(t);
  }

  it = SensorsIn.find("joint-angles");
  if (it!=SensorsIn.end())
  {
    const vector<double>& joint_anglesIn = it->second.getValues();
    joint_angles_.resize (joint_anglesIn.size () );
    for (unsigned i = 0; i < joint_anglesIn.size(); ++i)
      joint_angles_ (i) = joint_anglesIn[i];
    joint_anglesSOUT_.setConstant(joint_angles_);
    joint_anglesSOUT_.setTime(t);
  }

}

void PinocchioDevice::
setSensorsVelocities(map<string,dgsot::SensorValues> &SensorsIn,
		     int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  
  it = SensorsIn.find("velocities");
  if (it!=SensorsIn.end())
  {
    const vector<double>& velocitiesIn = it->second.getValues();
    dgRobotVelocity_.resize (velocitiesIn.size () + 6);
    for (unsigned i = 0; i < 6; ++i)
      dgRobotVelocity_ (i) = 0.;
    for (unsigned i = 0; i < velocitiesIn.size(); ++i)
      {
	dgRobotVelocity_ (i + 6) = velocitiesIn[i];
      }
    robotVelocity_.setConstant(dgRobotVelocity_);
    robotVelocity_.setTime(t);
  }
  
}

void PinocchioDevice::setSensorsTorquesCurrents(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("torques");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& torques = SensorsIn["torques"].getValues();
    torques_.resize(torques.size());
    for(std::size_t i = 0; i < torques.size(); ++i)
      torques_ (i) = torques [i];
    pseudoTorqueSOUT_.setConstant(torques_);
    pseudoTorqueSOUT_.setTime(t);
  }
  
  it = SensorsIn.find("currents");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& currents = SensorsIn["currents"].getValues();
    currents_.resize(currents.size());
    for(std::size_t i = 0; i < currents.size(); ++i)
      currents_ (i) = currents[i];
    currentsSOUT_.setConstant(currents_);
    currentsSOUT_.setTime(t);
  }
}

void PinocchioDevice::setSensorsGains(map<string,dgsot::SensorValues> &SensorsIn,
				     int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("p_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& p_gains = SensorsIn["p_gains"].getValues();
    p_gains_.resize(p_gains.size());
    for(std::size_t i = 0; i < p_gains.size(); ++i)
      p_gains_ (i) = p_gains[i];
    p_gainsSOUT_.setConstant(p_gains_);
    p_gainsSOUT_.setTime(t);
  }

  it = SensorsIn.find("d_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& d_gains = SensorsIn["d_gains"].getValues();
    d_gains_.resize(d_gains.size());
    for(std::size_t i = 0; i < d_gains.size(); ++i)
      d_gains_ (i) = d_gains[i];
    d_gainsSOUT_.setConstant(d_gains_);
    d_gainsSOUT_.setTime(t);
  }

}


void PinocchioDevice::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,dgsot::SensorValues>::iterator it;
  int t = stateSOUT_.getTime () + 1;

  setSensorsForce(SensorsIn,t);
  setSensorsIMU(SensorsIn,t);
  setSensorsEncoders(SensorsIn,t);
  setSensorsVelocities(SensorsIn,t);
  setSensorsTorquesCurrents(SensorsIn,t);
  setSensorsGains(SensorsIn,t);

  sotDEBUGOUT(25);
}

void PinocchioDevice::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void PinocchioDevice::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}


void PinocchioDevice::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void PinocchioDevice::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;
  vector<double> anglesOut;
  anglesOut.resize(state_.size());
  
  // Integrate control
  increment(timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << ((previousState_.size() == state_.size())?
				  (state_ - previousState_) : state_ ) 
		<< std::endl;
  ODEBUG5FULL("state = "<< state_);
  ODEBUG5FULL("diff  = " << ((previousState_.size() == state_.size())?
			     (state_ - previousState_) : state_ ) );
  previousState_ = state_;

  // Specify the joint values for the controller.
  if ((int)anglesOut.size()!=state_.size()-6)
    anglesOut.resize(state_.size()-6);

  for(unsigned int i=6; i < state_.size();++i)
    anglesOut[i-6] = state_(i);
  controlOut["control"].setValues(anglesOut);

  // Update position of freeflyer in global frame
  Eigen::Vector3d transq_(freeFlyerPose().translation());
  dg::sot::VectorQuaternion qt_(freeFlyerPose().linear());

  //translation
  for(int i=0; i<3; i++) baseff_[i] = transq_(i);
  
  //rotation: quaternion
  baseff_[3] = qt_.w();
  baseff_[4] = qt_.x();
  baseff_[5] = qt_.y();
  baseff_[6] = qt_.z();

  controlOut["baseff"].setValues(baseff_);
  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}

