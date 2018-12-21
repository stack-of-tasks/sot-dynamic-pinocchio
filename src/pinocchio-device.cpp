/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
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
  for( unsigned int i=0; i<4; ++i ) {
    delete forcesSOUT[i];
  }
}

PinocchioDevice::
PinocchioDevice( const std::string& n )
  :Entity(n)
  ,position_(6)
  ,sanityCheck_(true)
  ,controlSIN( NULL,"PinocchioDevice("+n+")::input(double)::control" )   
  ,attitudeSIN(NULL,"PinocchioDevice("+ n +")::input(vector3)::attitudeIN")
  ,zmpSIN(NULL,"PinocchioDevice("+n+")::input(vector3)::zmp")
  ,stateSOUT( "PinocchioDevice("+n+")::output(vector)::state" )   
  //,attitudeSIN(NULL,"PinocchioDevice::input(matrixRot)::attitudeIN")
  ,velocitySOUT( "PinocchioDevice("+n+")::output(vector)::velocity"  )
  ,attitudeSOUT( "PinocchioDevice("+n+")::output(matrixRot)::attitude" )
  ,motorcontrolSOUT   ( "PinocchioDevice("+n+")::output(vector)::motorcontrol" )
  ,previousControlSOUT( "PinocchioDevice("+n+")::output(vector)::previousControl" )
  ,ZMPPreviousControllerSOUT( "PinocchioDevice("+n+")::output(vector)::zmppreviouscontroller" )

  ,robotState_     ("PinocchioDevice("+n+")::output(vector)::robotState")
  ,robotVelocity_  ("PinocchioDevice("+n+")::output(vector)::robotVelocity")
  ,pseudoTorqueSOUT("PinocchioDevice("+n+")::output(vector)::ptorque" )

  ,ffPose_()
  ,forceZero6 (6)
{
  forceZero6.fill (0);
  /* --- SIGNALS --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
      new Signal<Vector, int>("PinocchioDevice("+n+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
      new Signal<Vector, int>("PinocchioDevice("+n+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
      new Signal<Vector, int>("PinocchioDevice("+n+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
      new Signal<Vector, int>("PinocchioDevice("+n+")::output(vector6)::forceLARM");

  signalRegistration( controlSIN
		      << stateSOUT
		      << robotState_
		      << robotVelocity_
                      << velocitySOUT
		      << attitudeSOUT
                      << attitudeSIN
		      << zmpSIN
		      << *forcesSOUT[0]
		      << *forcesSOUT[1]
                      << *forcesSOUT[2]
		      << *forcesSOUT[3]
		      << previousControlSOUT
                      << pseudoTorqueSOUT
		      << motorcontrolSOUT
		      << ZMPPreviousControllerSOUT );
  position_.fill(.0); stateSOUT.setConstant( position_ );

  velocity_.resize(position_.size()); velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );

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
  stateSOUT .setConstant( position_ );
  motorcontrolSOUT .setConstant( position_ );
}

void PinocchioDevice::
setVelocity( const Vector& vel )
{
  velocity_ = vel;
  velocitySOUT .setConstant( velocity_ );
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
  int time = stateSOUT.getTime();
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
  stateSOUT .setConstant( position_ ); stateSOUT.setTime( time+1 );

  for( int i=0;i<4;++i ){
    if(  !withForceSignals[i] ) forcesSOUT[i]->setConstant(forceZero6);
  }
  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );

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
  motorcontrolSOUT .setConstant( position_ );
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
  
  JointSHControlType_iterator it_control_type;
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
      
      stateSOUT .setConstant( position_ );
      velocitySOUT .setConstant( velocity_ );
      
    }
}


/* --- DISPLAY ------------------------------------------------------------ */

void PinocchioDevice::display ( std::ostream& os ) const
{os <<name<<": "<<position_<<endl; }
