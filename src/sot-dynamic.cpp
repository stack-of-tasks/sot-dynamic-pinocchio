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
#include <sot/core/debug.hh>

#include <sot-dynamic/dynamic.h>

#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <dynamic-graph/all-commands.h>

#include "../src/dynamic-command.h"


using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dg::sot::Dynamic::CLASS_NAME = "Dynamic";

Dynamic::
Dynamic( const std::string & name)
  :Entity(name)
  ,m_model()
  ,m_data(NULL)
  ,m_urdfPath("")

  ,init(false)

  ,jointPositionSIN(NULL,"sotDynamic("+name+")::input(vector)::position")
  ,freeFlyerPositionSIN(NULL,"sotDynamic("+name+")::input(vector)::ffposition")
  ,jointVelocitySIN(NULL,"sotDynamic("+name+")::input(vector)::velocity")
  ,freeFlyerVelocitySIN(NULL,"sotDynamic("+name+")::input(vector)::ffvelocity")
  ,jointAccelerationSIN(NULL,"sotDynamic("+name+")::input(vector)::acceleration")
  ,freeFlyerAccelerationSIN(NULL,"sotDynamic("+name+")::input(vector)::ffacceleration")

  ,newtonEulerSINTERN( boost::bind(&Dynamic::computeNewtonEuler,this,_1,_2),
		       jointPositionSIN<<freeFlyerPositionSIN
		       <<jointVelocitySIN<<freeFlyerVelocitySIN
		       <<jointAccelerationSIN<<freeFlyerAccelerationSIN,
		       "sotDynamic("+name+")::intern(dummy)::newtoneuler" )
   
  ,zmpSOUT( boost::bind(&Dynamic::computeZmp,this,_1,_2),
	    newtonEulerSINTERN,
	    "sotDynamic("+name+")::output(vector)::zmp" )
  ,JcomSOUT( boost::bind(&Dynamic::computeJcom,this,_1,_2),
	     newtonEulerSINTERN,
	     "sotDynamic("+name+")::output(matrix)::Jcom" )
  ,comSOUT( boost::bind(&Dynamic::computeCom,this,_1,_2),
	    newtonEulerSINTERN,
	    "sotDynamic("+name+")::output(vector)::com" )
  ,inertiaSOUT( boost::bind(&Dynamic::computeInertia,this,_1,_2),
		newtonEulerSINTERN,
		"sotDynamic("+name+")::output(matrix)::inertia" )
  ,footHeightSOUT( boost::bind(&Dynamic::computeFootHeight,this,_1,_2),
		   newtonEulerSINTERN,
		   "sotDynamic("+name+")::output(double)::footHeight" )
   
  ,upperJlSOUT( boost::bind(&Dynamic::getUpperPositionLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::upperJl" )
   
  ,lowerJlSOUT( boost::bind(&Dynamic::getLowerPositionLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::lowerJl" )
   
  ,upperVlSOUT( boost::bind(&Dynamic::getUpperVelocityLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::upperVl" )
   
  ,upperTlSOUT( boost::bind(&Dynamic::getMaxEffortLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::upperTl" )
   
  ,inertiaRotorSOUT( "sotDynamic("+name+")::output(matrix)::inertiaRotor" )
  ,gearRatioSOUT( "sotDynamic("+name+")::output(matrix)::gearRatio" )
  ,inertiaRealSOUT( boost::bind(&Dynamic::computeInertiaReal,this,_1,_2),
		    inertiaSOUT << gearRatioSOUT << inertiaRotorSOUT,
		    "sotDynamic("+name+")::output(matrix)::inertiaReal" )
  ,MomentaSOUT( boost::bind(&Dynamic::computeMomenta,this,_1,_2),
		inertiaSOUT,
		"sotDynamic("+name+")::output(vector)::momenta" )
  ,AngularMomentumSOUT( boost::bind(&Dynamic::computeAngularMomentum,this,_1,_2),
			inertiaSOUT,
			"sotDynamic("+name+")::output(vector)::angularmomentum" )
  ,dynamicDriftSOUT( boost::bind(&Dynamic::computeTorqueDrift,this,_1,_2),
		     newtonEulerSINTERN,
		     "sotDynamic("+name+")::output(vector)::dynamicDrift" )
{

  sotDEBUGIN(5);

  jointTypes.push_back("JointModelRX");
  jointTypes.push_back("JointModelRY");
  jointTypes.push_back("JointModelRZ");
  jointTypes.push_back("JointModelRevoluteUnaligned");
  jointTypes.push_back("JointModelSpherical");
  jointTypes.push_back("JointModelSphericalZYX");
  jointTypes.push_back("JointModelPX");
  jointTypes.push_back("JointModelPY");
  jointTypes.push_back("JointModelPZ");
  jointTypes.push_back("JointModelFreeFlyer");
  jointTypes.push_back("JointModelPlanar");
  jointTypes.push_back("JointModelTranslation");



  //TODO: gaze_joint is fixed joint: not one of pinocchio joint types. Confirm specificities joint.
  /*
  specificitiesMap["right-wrist"] = "RWristPitch";
  specificitiesMap["left-wrist"] = "LWristPitch";
  specificitiesMap["right-ankle"] = "LAnkleRoll";
  specificitiesMap["left-ankle"] = "RAnkleRoll";
  specificitiesMap["gaze"] = "HeadRoll";
  specificitiesMap["waist"] = "waist";
  specificitiesMap["chest"] = "TrunkYaw";
  */

  //TODO-------------------------------------------
 
  //if( build ) buildModel();
  //firstSINTERN.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
  //DEBUG: Why =0? should be function. firstSINTERN.setConstant(0);
  //endTODO--------------------------------------------

  signalRegistration(jointPositionSIN);
  signalRegistration(freeFlyerPositionSIN);
  signalRegistration(jointVelocitySIN);
  signalRegistration(freeFlyerVelocitySIN);
  signalRegistration(jointAccelerationSIN);
  signalRegistration(freeFlyerAccelerationSIN);
  signalRegistration(zmpSOUT);
  signalRegistration(comSOUT);
  signalRegistration(JcomSOUT);
  signalRegistration(footHeightSOUT);
  signalRegistration(upperJlSOUT);
  signalRegistration(lowerJlSOUT);
  signalRegistration(upperVlSOUT);
  signalRegistration(upperTlSOUT);
  signalRegistration(inertiaSOUT);
  signalRegistration(inertiaRealSOUT);
  signalRegistration(inertiaRotorSOUT);
  signalRegistration(gearRatioSOUT);
  signalRegistration(MomentaSOUT);
  signalRegistration(AngularMomentumSOUT);
  signalRegistration(dynamicDriftSOUT);

  //
  // Commands
  //
  std::string docstring;
  // setFiles
  docstring =
    "\n"
    "    Define files to parse in order to build the robot.\n"
    "\n"
    "      Input:\n"
    "        - a string: urdf file \n"
    "\n";
  addCommand("setFile",
	     new command::SetFile(*this, docstring));

  docstring =
    "\n"
    "    Parse files in order to build the robot.\n"
    "\n"
    "      Input:\n"
    "        - none \n"
    "\n";
  addCommand("parse",
	     new command::Parse(*this, docstring));

  docstring =
    "\n"
    "    Display the current robot configuration.\n"
    "\n"
    "      Input:\n"
    "        - none \n"
    "\n";
  addCommand("displayModel",
	     new command::DisplayModel(*this, docstring));
  {
    using namespace ::dg::command;
    // CreateOpPoint
    //TODO add operational joints
    docstring = "    \n"
      "    Create an operational point attached to a robot joint local frame.\n"
      "    \n"
      "      Input: \n"
      "        - a string: name of the operational point,\n"
      "        - a string: name the joint, or among (gaze, left-ankle, right ankle\n"
      "          , left-wrist, right-wrist, waist, chest).\n"
      "\n";
    addCommand("createOpPoint",
	       makeCommandVoid2(*this,&Dynamic::cmd_createOpPointSignals,
				docstring));
    
    docstring = docCommandVoid2("Create a jacobian (world frame) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createJacobian",
	       makeCommandVoid2(*this,&Dynamic::cmd_createJacobianWorldSignal,
				docstring));
    
    docstring = docCommandVoid2("Create a jacobian (endeff frame) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createJacobianEndEff",
	       makeCommandVoid2(*this,&Dynamic::cmd_createJacobianEndEffectorSignal,
				docstring));
    
    docstring = docCommandVoid2("Create a position (matrix homo) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createPosition",
	       makeCommandVoid2(*this,&Dynamic::cmd_createPositionSignal,docstring));
  }
  
  docstring = "    \n"
    "    Create an empty robot\n"
    "    \n";
  addCommand("createRobot", new command::CreateRobot(*this, docstring));
  
  docstring = "    \n"
    "    Create a joint\n"
    "    \n"
    "      Input:\n"
    "        - a string: name of the joint,\n"
    "        - a string: type of the joint in ['JointModelRX', 'JointModelRY', 'JointModelRZ', 'JointModelRevoluteUnaligned', 'JointModelSpherical', 'JointModelSphericalZYX', 'JointModelPX', 'JointModelPY', 'JointModelPZ', 'JointModelFreeFlyer', 'JointModelPlanar', 'JointModelTranslation'],\n"
    "        - a matrix: affine position of the joint.\n"
    "    \n";
  addCommand("createJoint", new command::CreateJoint(*this, docstring));
  
  docstring = "    \n"
    "    Add a child Body\n"
    "    \n"
    "      Input:\n"
    "        - a string: name of the parent body,\n"
    "        - a string: name of the joint. Joint must be newly created with CreateJoint,\n "
    "        - a string: name of the child body,\n"
    "        - a double: mass of the child body. Default = 0,\n"
    "        - a vector: com position of the child body. Default = Zero Vector,\n"
    "        - a matrix: 3x3 inertia matrix of the body. Default = Zero Matrix,\n"
    "    \n";
  addCommand("addBody", new command::AddBody(*this, docstring));
  
  docstring = "    \n"
    "    Set the mass of the body \n"
    "    \n"
    "      Input:\n"
    "        - a string:  name of the body whose properties are being set"
    "        - a double:  mass of the body."
    "    \n";
  addCommand("setMass", new command::SetMass(*this, docstring));

  docstring = "    \n"
    "    Get the dimension of the robot configuration.\n"
    "    \n"
    "      Return:\n"
    "        an unsigned int: the dimension.\n"
    "    \n";
  addCommand("getDimension",
	     new command::GetDimension(*this, docstring));
  
  docstring = "    \n"
    "    Write the robot kinematic chain in a file.\n"
    "    \n"
    "      Input:\n"
    "        a string: a filename.\n"
    "    \n";
  addCommand("write",
	     new command::Write(*this, docstring));
  //TODO: add specifities commands

  /*
  docstring = "    \n"
    "    Get ankle position in left foot frame.\n"
    "    \n";
  addCommand("getAnklePositionInFootFrame",
	     new dynamicgraph::command::Getter<Dynamic, dynamicgraph::Vector>
	     (*this, &Dynamic::getAnklePositionInFootFrame, docstring));
  
  */
  docstring = "    \n"
    "    Set the position of the center of mass of a body\n"
    "    \n"
    "      Input:\n"
    "        - a string:  name of the body whose properties are being set"
    "        - a vector:  local com position of the body."
    "    \n";
  addCommand("setLocalCenterOfMass", new command::SetLocalCenterOfMass(*this, docstring));

  docstring = "    \n"
    "    Set inertia matrix of a body attached to a joint\n"
    "    \n"
    "      Input:\n"
    "        - a string: name of the joint,\n"
    "        - a matrix: inertia matrix.\n"
    "    \n";
  addCommand("setInertiaMatrix",
	     new command::SetInertiaMatrix(*this, docstring));  

  docstring = "    \n"
    "    Set Inertia properties of a body\n"
    "    \n"
    "      Input:\n"
    "        - a string:  name of the body whose properties are being set"
    "        - a double:  mass of the body. default 0."
    "        - a vector:  com position of the body. default zero vector,"
    "        - a matrix:  inertia matrix of the body. default zero matrix."
    "    \n";
  addCommand("setInertiaProperties", new command::SetInertiaProperties(*this, docstring));


  docstring = "    \n"
    "    (Deprecated. Please use setLowerPositionLimit and setUpperPositionLimit)\n"
    "    Set the bounds of a joint degree of freedom\n"
    "    \n"
    "      Input:\n"
    "        - a string: the name of the joint,\n"
    "        - a non-negative integer: the dof id in the joint(0,1,2..)\n"
    "        - a floating point number: the minimal value,\n"
    "        - a floating point number: the maximal value.\n"
    "    \n";
  addCommand("setDofBounds", new command::SetDofBounds(*this, docstring));

  docstring = "    \n"
    "    Set the lower bounds of the joint position\n"
    "    \n"
    "      Input:\n"
    "        - a string: the name of the joint,\n"
    "        - a vector: lower limit bounds for all dofs of joint,\n"
    "    \n";
  addCommand("setLowerPositionLimit", new command::SetLowerPositionLimit(*this, docstring));
  
  docstring = "    \n"
    "    Set the upper bounds of the joint position\n"
    "    \n"
    "      Input:\n"
    "        - a string: the name of the joint,\n"
    "        - a vector: upper limit bounds for all dofs of joint,\n"
    "    \n";
  addCommand("setUpperPositionLimit", new command::SetUpperPositionLimit(*this, docstring));
  
  docstring = "    \n"
    "    Set the upper bounds of the joint velocity\n"
    "    \n"
    "      Input:\n"
    "        - a string: the name of the joint,\n"
    "        - a vector: upper limit bounds for joint velocities,\n"
    "    \n";
  addCommand("setMaxVelocityLimit", new command::SetMaxVelocityLimit(*this, docstring));
  
  docstring = "    \n"
    "    Set the upper bounds of the joint effort\n"
    "    \n"
    "      Input:\n"
    "        - a string: the name of the joint,\n"
    "        - a vector: upper limit bounds for joint effort,\n"
    "    \n";
  addCommand("setMaxEffortLimit", new command::SetMaxEffortLimit(*this, docstring));
  sotDEBUG(10)<< "Dynamic class_name address"<<&CLASS_NAME<<std::endl;
  
  sotDEBUGOUT(5);
}


Dynamic::~Dynamic( void ) {
  sotDEBUGIN(15);
    //  if (0!=m_model){ delete m_model; m_model=NULL;}
  if (0!=m_data){ delete m_data; m_data=NULL;}
  for( std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
       iter != genericSignalRefs.end();
       ++iter) {
    SignalBase<int>* sigPtr = *iter;
    delete sigPtr;
  }
  
  sotDEBUGOUT(15);
}

/* ---------------- CONFIG -------------------------------------------- */
//Import from urdf file
void Dynamic::setUrdfFile(const std::string& filename) {
  sotDEBUGIN(15);
  m_urdfPath = filename;
  sotDEBUG(15)<<"Urdf file path: "<<m_urdfPath<<std::endl;
  sotDEBUGOUT(15);
}

void Dynamic::parseUrdfFile() {
  sotDEBUGIN(15);
  if (m_urdfPath.empty()) std::cerr<<"Set input file first"<<std::endl;
  m_model = se3::urdf::buildModel(this->m_urdfPath, false);
  if (m_data) delete m_data;
  m_data = new se3::Data(m_model);
  init=true;
  std::vector<se3::Frame>::const_iterator it = m_model.operational_frames.begin();
  for(;it!=m_model.operational_frames.end();it++)
    sotDEBUG(15)<<"Operational Frames Added: "<<it->name<<std::endl;
  sotDEBUG(15)<<m_model<<std::endl;
  displayModel();
  sotDEBUGOUT(15);
}


//Create an empty robot
void Dynamic::createRobot()
{
  sotDEBUGIN(15);
  delete m_data;
  m_model= se3::Model();
  m_data = new se3::Data(m_model);
  init=true;
  sotDEBUGOUT(15);
}


void Dynamic::createJoint(const std::string& inJointName,
			  const std::string& inJointType,
			  const dg::Matrix& inPosition) {
  sotDEBUGIN(15);
  if (jointMap_.count(inJointName) >= 1)
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "a joint with name " + inJointName +
			       " has already been created.");
  if (m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "a joint with name " + inJointName +
			       " already exists in the model.");

  if(std::find(jointTypes.begin(),jointTypes.end(),inJointType) != jointTypes.end()) {
    Eigen::Matrix4d inposition_ = inPosition;
    JointDetails jointDetails(inJointType,inposition_);
    jointMap_[inJointName] = jointDetails;
  }
  else {
    std::vector<std::string>::iterator it;    std::stringstream ss;
    for (it = jointTypes.begin(); it != jointTypes.end(); ++it)  ss << *it;
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       inJointType + " is not a valid type. Valid types are:" +ss.str());
  sotDEBUGOUT(15);
  }
}


void Dynamic::addBody(const std::string& inParentName,
		      const std::string& inJointName,
		      const std::string& inChildName) {
  sotDEBUGIN(15);
  addBody(inParentName,inJointName,inChildName
	  ,0,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Zero());
  sotDEBUGOUT(15);
}

void Dynamic::addBody(const std::string& inParentName,
		      const std::string& inJointName,
		      const std::string& inChildName,
		      const double mass,
		      const dg::Vector& lever,
		      const dg::Matrix& inertia3) {
  sotDEBUGIN(15);
  if (jointMap_.count(inJointName) != 1)
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  if (m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "A joint with name " + inJointName +
			       " already exists in the model.");
  if (!m_model.existBodyName(inParentName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No parent body with name " + inParentName +
			       " has been created.");
  se3::Model::Index parent = m_model.getBodyId(inParentName);
  const JointDetails jointDetails_ = jointMap_[inJointName];
  const std::string type = jointDetails_.first;
  const se3::Inertia inertia(mass,lever,inertia3);
  if(type == "JointModelRX")
    m_model.addBody(parent,se3::JointModelRX (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type == "JointModelRY" )
    m_model.addBody(parent,se3::JointModelRY (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelRZ" ) {
    m_model.addBody(parent,se3::JointModelRZ (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  }
  else if(type =="JointModelRevoluteUnaligned" )
    m_model.addBody(parent,se3::JointModelRevoluteUnaligned (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelSpherical" )
    m_model.addBody(parent,se3::JointModelSpherical (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelSphericalZYX" )
    m_model.addBody(parent,se3::JointModelSphericalZYX (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelPX" )
    m_model.addBody(parent,se3::JointModelPX (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelPY" )
    m_model.addBody(parent,se3::JointModelPY (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelPZ" )
    m_model.addBody(parent,se3::JointModelPZ (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelFreeFlyer" )
    m_model.addBody(parent,se3::JointModelFreeFlyer (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelPlanar" )
    m_model.addBody(parent,se3::JointModelPlanar (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else if(type =="JointModelTranslation" )
    m_model.addBody(parent,se3::JointModelTranslation (),
			  jointDetails_.second,inertia,inJointName,inChildName);
  else
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "Joint with type " + type +
			       "should not have been created");

  if(m_data) delete m_data;
  m_data = new se3::Data(m_model);
  sotDEBUGOUT(15);
}

  /*--------------------------------SETTERS-------------------------------------------*/

void Dynamic::setMass(const std::string& inBodyName,
		      const double mass) {
  sotDEBUGIN(15);
  if (!m_model.existBodyName(inBodyName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No body with name " + inBodyName +
			       " has been added.");
  se3::Model::Index index = m_model.getBodyId(inBodyName);
  m_model.inertias[index].mass() = mass;
  sotDEBUGOUT(15);
}

void Dynamic::setLocalCenterOfMass(const std::string& inBodyName,
				   const dg::Vector& lever) {
  sotDEBUGIN(15);
  if (!m_model.existBodyName(inBodyName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No body with name " + inBodyName +
			       " has been added.");
  se3::Model::Index index = m_model.getBodyId(inBodyName);
  m_model.inertias[index].lever() = lever;
  sotDEBUGOUT(15);
}

void Dynamic::setInertiaMatrix(const std::string& inBodyName,
			       const dg::Matrix& inertia3) {
  sotDEBUGIN(15);
  if (!m_model.existBodyName(inBodyName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No body with name " + inBodyName +
			       " has been added.");
  se3::Model::Index index = m_model.getBodyId(inBodyName);
  se3::Symmetric3 symmetricMatrix(inertia3);
  m_model.inertias[index].inertia() = symmetricMatrix;
  sotDEBUGOUT(15);
}

void Dynamic::setInertiaProperties(const std::string& inBodyName,
				   const double mass,
				   const dg::Vector& lever,
				   const dg::Matrix& inertia3) {
  sotDEBUGIN(15);
  if (!m_model.existBodyName(inBodyName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No body with name " + inBodyName +
			       " has been added.");
  se3::Inertia inertia_(mass,lever,inertia3);
  se3::Model::Index index = m_model.getBodyId(inBodyName);
  m_model.inertias[index] = inertia_;
  sotDEBUGOUT(15);
}


void Dynamic::setDofBounds(const std::string& inJointName,
			   const unsigned int inDofId,
			   const double inMinValue, double inMaxValue) {
  sotDEBUGIN(15);
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nq = se3::idx_q(m_model.joints[joint_index]);  

  assert(se3::nq(m_model.joints[joint_index]) > inDofId);

  m_model.lowerPositionLimit(prev_cumulative_nq+inDofId) = inMinValue;
  m_model.upperPositionLimit(prev_cumulative_nq+inDofId) = inMaxValue;
  sotDEBUGOUT(15);  
  return;
}

void Dynamic::setLowerPositionLimit(const std::string& inJointName,
				    const double lowPos) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);

  int prev_cumulative_nq = se3::idx_q(m_model.joints[joint_index]);  
  assert (se3::nq(m_model.joints[joint_index])==1 && "Joint is not revolute");
  m_model.lowerPositionLimit(prev_cumulative_nq) = lowPos;
  return;
}

void Dynamic::setLowerPositionLimit(const std::string& inJointName,
				    const dg::Vector& lowPos) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nq = se3::idx_q(m_model.joints[joint_index]);  
  int current_nq = se3::nq(m_model.joints[joint_index]);
  assert (lowPos.size()==current_nq);
  m_model.lowerPositionLimit.segment(prev_cumulative_nq,current_nq) = lowPos;
  return;
}


void Dynamic::setUpperPositionLimit(const std::string& inJointName,
				    const double upPos) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nq = se3::idx_q(m_model.joints[joint_index]);  
  assert (se3::nq(m_model.joints[joint_index])==1 && "Joint is not revolute");
  m_model.upperPositionLimit(prev_cumulative_nq) = upPos;
  return;
}

void Dynamic::setUpperPositionLimit(const std::string& inJointName,
				    const dg::Vector& upPos) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nq = se3::idx_q(m_model.joints[joint_index]);  
  int current_nq = se3::nq(m_model.joints[joint_index]);

  assert (upPos.size()==current_nq);
  m_model.upperPositionLimit.segment(prev_cumulative_nq,current_nq) = upPos;
  return;
}

void Dynamic::setMaxVelocityLimit(const std::string& inJointName,
				  const double maxVel) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nv = se3::idx_v(m_model.joints[joint_index]);  

  assert (se3::nv(m_model.joints[joint_index])==1 && "Joint is not revolute");
  m_model.velocityLimit(prev_cumulative_nv) = maxVel;
  return;
}

void Dynamic::setMaxVelocityLimit(const std::string& inJointName,
				  const dg::Vector& maxVel) {
  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nv = se3::idx_v(m_model.joints[joint_index]);  
  int current_nv = se3::nv(m_model.joints[joint_index]);
  assert (maxVel.size()==current_nv);
  m_model.velocityLimit.segment(prev_cumulative_nv,current_nv) = maxVel;
  return;
}


void Dynamic::setMaxEffortLimit(const std::string& inJointName,
				const double maxEffort) {

  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nv = se3::idx_v(m_model.joints[joint_index]);  
  assert (se3::nv(m_model.joints[joint_index])==1 && "Joint is not revolute");
  m_model.effortLimit(prev_cumulative_nv) = maxEffort;
  return;
}

void Dynamic::setMaxEffortLimit(const std::string& inJointName,
				const dg::Vector& maxEffort) {

  if (!m_model.existJointName(inJointName))
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  se3::Model::Index joint_index = m_model.getJointId(inJointName);
  int prev_cumulative_nv = se3::idx_v(m_model.joints[joint_index]);  
  int current_nv = se3::nv(m_model.joints[joint_index]);
  assert (maxEffort.size()==current_nv);
  m_model.effortLimit.segment(prev_cumulative_nv,current_nv) = maxEffort;
  return;
}

/*--------------------------------GETTERS-------------------------------------------*/


dg::Vector& Dynamic::
getLowerPositionLimits(dg::Vector& res, const int&)
{
 
  sotDEBUGIN(15);
  res.resize(m_model.nq);
  res = m_model.lowerPositionLimit;
  sotDEBUG(15) << "lowerLimit (" << res << ")=" << std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getUpperPositionLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  res.resize(m_model.nq);
  res = m_model.upperPositionLimit;
  sotDEBUG(15) << "upperLimit (" << res << ")=" <<std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getUpperVelocityLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  res.resize(m_model.nv);
  res = m_model.velocityLimit;
  sotDEBUG(15) << "upperVelocityLimit (" << res << ")=" <<std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getMaxEffortLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  res.resize(m_model.nv);
  res = m_model.effortLimit;
  sotDEBUGOUT(15);
  return res;
}


//TODO: To be set via srdf file
/*dynamicgraph::Vector Dynamic::getAnklePositionInFootFrame() const
{
  if (!m_data) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
			       "you must create a robot first.");
  }
  
  dg::Vector anklePosition(3);
  foot->getAnklePositionInLocalFrame(anklePosition);
  dynamicgraph::Vector res(3);
  res(0) = anklePosition[0];
  res(1) = anklePosition[1];
  res(2) = anklePosition[2];
  return res;
}
*/

/* ---------------- INTERNAL ------------------------------------------------ */
dg::Vector Dynamic::getPinocchioPos(int time)
{
  sotDEBUGIN(15);
  dg::Vector qJoints=jointPositionSIN.access(time);
  dg::Vector q;
  
  if( freeFlyerPositionSIN) {
    dg::Vector qFF=freeFlyerPositionSIN.access(time);
    q.resize(qJoints.size() + 7);
    urdf::Rotation rot;
    rot.setFromRPY(qFF(3),qFF(4),qFF(5));
    double x,y,z,w;
    rot.getQuaternion(x,y,z,w);
    q << qFF(0),qFF(1),qFF(2),x,y,z,w,qJoints;
  }
  else if (se3::nv(m_model.joints[1]) == 6){
    dg::Vector qFF = qJoints.head<6>();
    urdf::Rotation rot;
    rot.setFromRPY(qFF(3),qFF(4),qFF(5));
    double x,y,z,w;
    rot.getQuaternion(x,y,z,w);
    q.resize(qJoints.size()+1);
    q << qFF(0),qFF(1),qFF(2),x,y,z,w,qJoints.segment(6,qJoints.size()-6);
    assert(q.size() == m_model.nq);
  }
  else {
    q.resize(qJoints.size());
    q=qJoints;
  }
  sotDEBUGOUT(15) <<"Position out"<<q<<std::endl;
  return q;
}

Eigen::VectorXd Dynamic::getPinocchioVel(int time)
{
  const Eigen::VectorXd vJoints=jointVelocitySIN.access(time);
  if(freeFlyerVelocitySIN){
    const Eigen::VectorXd vFF=freeFlyerVelocitySIN.access(time);
    Eigen::VectorXd v(vJoints.size() + vFF.size());
    v << vFF,vJoints;
    return v;
  }
  else return vJoints;
}

Eigen::VectorXd Dynamic::getPinocchioAcc(int time)
{
  const Eigen::VectorXd aJoints=jointAccelerationSIN.access(time);
  if(freeFlyerAccelerationSIN){
    const Eigen::VectorXd aFF=freeFlyerAccelerationSIN.access(time);
    Eigen::VectorXd a(aJoints.size() + aFF.size());
    a << aFF,aJoints;
    return a;
  }
  else return aJoints;
}

/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
dg::SignalTimeDependent< dg::Matrix,int > & Dynamic::
createJacobianSignal( const std::string& signame, const std::string& jointName )
{
  sotDEBUGIN(15);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model.existFrame(jointName)) {
    int frameId = m_model.getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericJacobian,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else if(m_model.existJointName(jointName)) {
    int jointId = m_model.getJointId(jointName); 
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericJacobian,this,false,jointId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< dg::Matrix,int > & Dynamic::
createEndeffJacobianSignal( const std::string& signame, const std::string& jointName )
{
  sotDEBUGIN(15);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model.existFrame(jointName)) {
    int frameId = m_model.getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericEndeffJacobian,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else if(m_model.existJointName(jointName)) {
    int jointId = m_model.getJointId(jointName); 
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericEndeffJacobian,this,false,jointId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< MatrixHomogeneous,int >& Dynamic::
createPositionSignal( const std::string& signame, const std::string& jointName)
{
  sotDEBUGIN(15);
  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig;
  if(m_model.existFrame(jointName)) {
    int frameId = m_model.getFrameId(jointName);
    sig = new dg::SignalTimeDependent< MatrixHomogeneous,int >
      ( boost::bind(&Dynamic::computeGenericPosition,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrixHomo)::"+signame );
  }
  else if(m_model.existJointName(jointName)) {
    int jointId = m_model.getJointId(jointName); 
    sig = new dg::SignalTimeDependent< MatrixHomogeneous,int >
      ( boost::bind(&Dynamic::computeGenericPosition,this,false,jointId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrixHomo)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);
  
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

SignalTimeDependent< dg::Vector,int >& Dynamic::
createVelocitySignal( const std::string& signame,const std::string& jointName )
{
  sotDEBUGIN(15);
  int jointId = m_model.getJointId(jointName);

  SignalTimeDependent< dg::Vector,int > * sig
    = new SignalTimeDependent< dg::Vector,int >
    ( boost::bind(&Dynamic::computeGenericVelocity,this,jointId,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(dg::Vector)::"+signame );
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< dg::Vector,int >& Dynamic::
createAccelerationSignal( const std::string& signame, const std::string& jointName)
{
  sotDEBUGIN(15);
  int jointId = m_model.getJointId(jointName);
  dg::SignalTimeDependent< dg::Vector,int > * sig
    = new dg::SignalTimeDependent< dg::Vector,int >
    ( boost::bind(&Dynamic::computeGenericAcceleration,this,jointId,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrixHomo)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}




void Dynamic::
destroyJacobianSignal( const std::string& signame )
{
  sotDEBUGIN(15);
  bool deletable = false;
  dg::SignalTimeDependent< dg::Matrix,int > * sig = & jacobiansSOUT( signame );
  for(std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
      iter != genericSignalRefs.end();
      ++iter) {
    if( (*iter) == sig ) {
      genericSignalRefs.erase(iter); deletable = true;
      break;
    }
  }

  if(! deletable )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::CANT_DESTROY_SIGNAL,
				     "Cannot destroy signal",
				     " (while trying to remove generic jac. signal <%s>).",
				     signame.c_str() );
    }
  signalDeregistration( signame );
  delete sig;
}

void Dynamic::
destroyPositionSignal( const std::string& signame )
{
  sotDEBUGIN(15);
  bool deletable = false;
  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig = & positionsSOUT( signame );
  for(  std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
	iter != genericSignalRefs.end();
	++iter )
    {
      if( (*iter) == sig ) { genericSignalRefs.erase(iter); deletable = true; break; }
    }

  if(! deletable )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::CANT_DESTROY_SIGNAL,
				     "Cannot destroy signal",
				     " (while trying to remove generic pos. signal <%s>).",
				     signame.c_str() );
    }

  signalDeregistration( signame );

  delete sig;
}

void Dynamic::
destroyVelocitySignal( const std::string& signame )
{
  sotDEBUGIN(15);
  bool deletable = false;
  SignalTimeDependent< dg::Vector,int > * sig = & velocitiesSOUT( signame );
  for(  std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
	iter != genericSignalRefs.end();
	++iter )
    {
      if( (*iter) == sig ) { genericSignalRefs.erase(iter); deletable = true; break; }
    }

  if(! deletable )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::CANT_DESTROY_SIGNAL,
				     "Cannot destroy signal",
				     " (while trying to remove generic pos. signal <%s>).",
				     signame.c_str() );
    }

  signalDeregistration( signame );

  delete sig;
}

void Dynamic::
destroyAccelerationSignal( const std::string& signame )
{
  sotDEBUGIN(15);
  bool deletable = false;
  dg::SignalTimeDependent< dg::Vector,int > * sig = & accelerationsSOUT( signame );
  for(  std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
	iter != genericSignalRefs.end();
	++iter )
    {
      if( (*iter) == sig ) { genericSignalRefs.erase(iter); deletable = true; break; }
    }

  if(! deletable )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::CANT_DESTROY_SIGNAL,
				  getName() + ":cannot destroy signal",
				  " (while trying to remove generic acc "
				  "signal <%s>).",
				  signame.c_str() );
    }

  signalDeregistration( signame );

  delete sig;
}

/* --------------------- COMPUTE ------------------------------------------------- */

dg::Vector& Dynamic::computeZmp( dg::Vector& res,int time )
{
    //TODO: To be verified
    sotDEBUGIN(25);
    if (res.size()!=3)
        res.resize(3);
    newtonEulerSINTERN(time);
    se3::Force ftau = m_data->oMi[1].act(m_data->f[1]);
    se3::Force::Vector3 tau = ftau.angular();
    se3::Force::Vector3 f = ftau.linear();
    res(0) = -tau[1]/f[2];
    res(1) = tau[0]/f[2];
    res(2) = 0;

    sotDEBUGOUT(25);

    return res;
}

//In world coordinates
dg::Matrix& Dynamic::
computeGenericJacobian(bool isFrame, int jointId, dg::Matrix& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  res.resize(6,m_model.nv);
  se3::computeJacobians(m_model,*m_data,this->getPinocchioPos(time));

  se3::Data::Matrix6x m_output = Eigen::MatrixXd::Zero(6,m_model.nv);
  //Computes Jacobian in world coordinates. 
  if(isFrame){
    se3::framesForwardKinematics(m_model,*m_data);
    se3::getFrameJacobian<false>(m_model,*m_data,(se3::Model::Index)jointId,m_output);
  }
  else se3::getJacobian<false>(m_model,*m_data,(se3::Model::Index)jointId,m_output);
  res = m_output;
  sotDEBUGOUT(25);
  return res;
}

dg::Matrix& Dynamic::
computeGenericEndeffJacobian(bool isFrame, int jointId,dg::Matrix& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  res.resize(6,m_model.nv);
  //In local coordinates.
  se3::computeJacobians(m_model,*m_data,this->getPinocchioPos(time));
  se3::Data::Matrix6x m_output = Eigen::MatrixXd::Zero(6,m_model.nv);

  if(isFrame){
    se3::framesForwardKinematics(m_model,*m_data);
    se3::getFrameJacobian<true>(m_model,*m_data,(se3::Model::Index)jointId,m_output);
  }
  else se3::getJacobian<true>(m_model,*m_data,(se3::Model::Index)jointId,m_output);

  res = m_output;
  sotDEBUGOUT(25);

  return res;
}

MatrixHomogeneous& Dynamic::
computeGenericPosition(bool isFrame, int jointId, MatrixHomogeneous& res, int time)
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  
  if(isFrame){
    //TODO: Confirm if we need this. Already being called when calculating jacobian
    //se3::framesForwardKinematics(m_model,*m_data);
    res.matrix()= m_data->oMof[jointId].toHomogeneousMatrix();
  }
  else res.matrix()= m_data->oMi[jointId].toHomogeneousMatrix();

  sotDEBUGOUT(25);
  return res;
}

dg::Vector& Dynamic::
computeGenericVelocity( int jointId, dg::Vector& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  res.resize(6);
  se3::Motion aRV = m_data->v[jointId];
  res<<aRV.linear(),aRV.angular();
  sotDEBUGOUT(25);
  return res;
}

dg::Vector& Dynamic::
computeGenericAcceleration( int jointId ,dg::Vector& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  res.resize(6);
  se3::Motion aRA = m_data->a[jointId];
  res<<aRA.linear(),aRA.angular();
  sotDEBUGOUT(25);
  return res;
}

int& Dynamic::
computeNewtonEuler( int& dummy,int time )
{
  sotDEBUGIN(15);

  const Eigen::VectorXd q=getPinocchioPos(time);
  const Eigen::VectorXd v=getPinocchioVel(time);
  const Eigen::VectorXd a=getPinocchioAcc(time);
  se3::rnea(m_model,*m_data,q,v,a);
  
  sotDEBUG(1)<< "pos = " <<q <<std::endl;
  sotDEBUG(1)<< "vel = " <<v <<std::endl;
  sotDEBUG(1)<< "acc = " <<a <<std::endl;

  sotDEBUGOUT(15);
  return dummy;
}

dg::Matrix& Dynamic::
computeJcom( dg::Matrix& Jcom,int time )
{
  
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  const Eigen::VectorXd q=getPinocchioPos(time);

  Jcom = se3::jacobianCenterOfMass(m_model, *m_data,
				   q, false);
  sotDEBUGOUT(25);
  return Jcom;
}

dg::Vector& Dynamic::
computeCom( dg::Vector& com,int time )
{

  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  const Eigen::VectorXd q=getPinocchioPos(time);
  com = se3::centerOfMass(m_model,*m_data,q,false,true);
  sotDEBUGOUT(25);
  return com;
}

dg::Matrix& Dynamic::
computeInertia( dg::Matrix& res,int time )
{
    sotDEBUGIN(25);
    newtonEulerSINTERN(time);
    //TODO: USE CCRBA
    dg::Matrix upperInertiaMatrix = se3::crba(m_model,
					       *m_data,this->getPinocchioPos(time)); 
    res = upperInertiaMatrix;
    res.triangularView<Eigen::StrictlyLower>() = upperInertiaMatrix.transpose().triangularView<Eigen::StrictlyLower>();
    sotDEBUGOUT(25);
    return res;
}

dg::Matrix& Dynamic::
computeInertiaReal( dg::Matrix& res,int time )
{
  sotDEBUGIN(25);

  const dg::Matrix & A = inertiaSOUT(time);
  const dg::Vector & gearRatio = gearRatioSOUT(time);
  const dg::Vector & inertiaRotor = inertiaRotorSOUT(time);

  res = A;
  for( int i=0;i<gearRatio.size();++i )
    res(i,i) += (gearRatio(i)*gearRatio(i)*inertiaRotor(i));

  sotDEBUGOUT(25);
  return res;
}

double& Dynamic::
computeFootHeight (double &res , int time)
{
  //Ankle position in local foot frame
  //TODO: Confirm that it is in the foot frame
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  if(!m_model.existJointName("r_sole_joint")) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
			       "Robot has no joint corresponding to rigthFoot");
  }
  int jointId = m_model.getJointId("r_sole_joint");
  Eigen::Vector3d anklePosInLocalRefFrame= m_data->liMi[jointId].translation();
  // TODO: positive or negative? Current output:negative
  res = anklePosInLocalRefFrame(2);
  sotDEBUGOUT(25);
  return res;
}

dg::Vector& Dynamic::
computeTorqueDrift( dg::Vector& tauDrift,const int  &time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  tauDrift = m_data->tau;
  sotDEBUGOUT(25);
  return tauDrift;
}

dg::Vector& Dynamic::
computeMomenta(dg::Vector & Momenta, int time)
{
  sotDEBUGIN(25);
  inertiaSOUT(time);
  if (Momenta.size()!=6)
    Momenta.resize(6);

  Momenta = m_data->hg.toVector_impl();

  sotDEBUGOUT(25) << "Momenta :" << Momenta ;
  return Momenta; 
}

dg::Vector& Dynamic::
computeAngularMomentum(dg::Vector & Momenta, int time)
{
  sotDEBUGIN(25);
  inertiaSOUT(time);

  if (Momenta.size()!=3)
    Momenta.resize(3);
  return Momenta;
  Momenta = m_data->hg.angular_impl();

  sotDEBUGOUT(25) << "AngularMomenta :" << Momenta ;
  return Momenta;
}

/* ------------------------ SIGNAL CASTING--------------------------------------- */

dg::SignalTimeDependent<dg::Matrix,int>& Dynamic::
jacobiansSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);
  try {
    dg::SignalTimeDependent<dg::Matrix,int>& res
      = dynamic_cast< dg::SignalTimeDependent<dg::Matrix,int>& >( sigabs );
    return res;
  } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type matrix.",
				  name.c_str());
  }
}
dg::SignalTimeDependent<MatrixHomogeneous,int>& Dynamic::
positionsSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);
  try {
    dg::SignalTimeDependent<MatrixHomogeneous,int>& res
      = dynamic_cast< dg::SignalTimeDependent<MatrixHomogeneous,int>& >( sigabs );
    return res;
  } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type matrixHomo.",
				  name.c_str());
  }
}

dg::SignalTimeDependent<dg::Vector,int>& Dynamic::
velocitiesSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);
  try {
    dg::SignalTimeDependent<dg::Vector,int>& res
      = dynamic_cast< dg::SignalTimeDependent<dg::Vector,int>& >( sigabs );
    return res;
 } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type Vector.",
				  name.c_str());
  }
}

dg::SignalTimeDependent<dg::Vector,int>& Dynamic::
accelerationsSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);
  try {
    dg::SignalTimeDependent<dg::Vector,int>& res
      = dynamic_cast< dg::SignalTimeDependent<dg::Vector,int>& >( sigabs );
    return res;
  } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type Vector.",
				  name.c_str());
  }
}

/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/

/* --- PARAMS --------------------------------------------------------------- */
void Dynamic::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "# In { Cmd " << cmdLine <<std::endl;
  std::string filename;
  if( cmdLine == "setFile" ) {
    cmdArgs>>filename; setUrdfFile(filename);
  }
  if( cmdLine == "displayModel" ) {
    displayModel();
  }
  else if( cmdLine == "parse" ) {
    if(!init)  parseUrdfFile();
    else std::cout << "  !! Already parsed." <<std::endl;
  }
  else if( cmdLine == "displayFile" ) {
    cmdArgs >> std::ws;
    os << m_urdfPath << std::endl;
  }
  else if( cmdLine == "createJacobian" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createJacobianSignal(signame,jointName);
  }
  else if( cmdLine == "destroyJacobian" ) {
    std::string Jname; cmdArgs >> Jname;
    destroyJacobianSignal(Jname);
  }
  else if( cmdLine == "createPosition" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createPositionSignal(signame,jointName);
  }
  else if( cmdLine == "destroyPosition" ) {
    std::string Jname; cmdArgs >> Jname;
    destroyPositionSignal(Jname);
  }
  else if( cmdLine == "createVelocity" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createVelocitySignal(signame,jointName);
  }
  else if( cmdLine == "destroyVelocity" ) {
    std::string Jname; cmdArgs >> Jname;
    destroyVelocitySignal(Jname);
  }
  else if( cmdLine == "createAcceleration" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createAccelerationSignal(signame,jointName);
  }
  else if( cmdLine == "destroyAcceleration" ) {
    std::string Jname; cmdArgs >> Jname;
    destroyAccelerationSignal(Jname);
  }
  else if( cmdLine == "createEndeffJacobian" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createEndeffJacobianSignal(signame,jointName);
  }
  else if( cmdLine == "createOpPoint" ) {
    std::string signame; cmdArgs >> signame;
    std::string jointName; cmdArgs >> jointName;
    createEndeffJacobianSignal(std::string("J")+signame,jointName);
    createPositionSignal(signame,jointName);
    sotDEBUG(15)<<std::endl;
  }
  else if( cmdLine == "destroyOpPoint" ) {
    std::string Jname; cmdArgs >> Jname;
    destroyJacobianSignal(std::string("J")+Jname);
    destroyPositionSignal(Jname);
  }
  else if( cmdLine == "ndof" ) {
    os << "Number of Degree of freedom:" <<m_data->J.cols() << std::endl;
    return;
  }
  else if( cmdLine == "help" ) {
    os << "Dynamics:"<<std::endl
       << "  - setFile <%1>\t:set files in the order cited above" <<std::endl
       << "  - displayModel\t:display the current model configuration" <<std::endl
       << "  - displayFile\t\t\t:display the urdf config file" <<std::endl
       << "  - parse\t\t\t:parse the files set unsing the set{Xml|Vrml} commands." <<std::endl
       << "  - createJacobian <name> <point>:create a signal named <name> " << std::endl
       << "  - destroyJacobian <name>\t:delete the jacobian signal <name>" << std::endl
       << "  - createEndeffJacobian <name> <point>:create a signal named <name> "
       << "forwarding the jacobian computed at <point>." <<std::endl
       << "  - {create|destroy}Position\t:handle position signals." <<std::endl
       << "  - {create|destroy}OpPoint\t:handle Operation Point (ie pos+jac) signals." <<std::endl
       << "  - {create|destroy}Acceleration\t:handle acceleration signals." <<std::endl
      /*TODO: Put these flags for computations (copied from humanoid_robot.py):
    def setProperties(self, model):
        model.setProperty('TimeStep', str(self.timeStep))
        model.setProperty('ComputeAcceleration', 'false')
        model.setProperty('ComputeAccelerationCoM', 'false')
        model.setProperty('ComputeBackwardDynamics', 'false')
        model.setProperty('ComputeCoM', 'false')
        model.setProperty('ComputeMomentum', 'false')
        model.setProperty('ComputeSkewCom', 'false')
        model.setProperty('ComputeVelocity', 'false')
        model.setProperty('ComputeZMP', 'false')

        model.setProperty('ComputeAccelerationCoM', 'true')
        model.setProperty('ComputeCoM', 'true')
        model.setProperty('ComputeVelocity', 'true')
        model.setProperty('ComputeZMP', 'true')

        if self.enableZmpComputation:
            model.setProperty('ComputeBackwardDynamics', 'true')
            model.setProperty('ComputeAcceleration', 'true')
            model.setProperty('ComputeMomentum', 'true')
      */
      //       << "  - {get|set}Property <name> [<val>]: set/get the property." <<std::endl
      //       << "  - displayProperties: print the prop-val couples list." <<std::endl
       << "  - ndof\t\t\t: display the number of DOF of the robot."<< std::endl;
    
    Entity::commandLine(cmdLine,cmdArgs,os);
  }
  else {
    Entity::commandLine( cmdLine,cmdArgs,os); }
  
  sotDEBUGOUT(15);
  
}

//jointName is either a fixed-joint (pinocchio operational frame) or a 
//movable joint (pinocchio joint-variant).
void Dynamic::cmd_createOpPointSignals( const std::string& opPointName,
					const std::string& jointName )
{
    createEndeffJacobianSignal(std::string("J")+opPointName,jointName);
    createPositionSignal(opPointName,jointName);
}
void Dynamic::cmd_createJacobianWorldSignal( const std::string& signalName,
					     const std::string& jointName )
{
    createJacobianSignal(signalName, jointName);
}
void Dynamic::cmd_createJacobianEndEffectorSignal( const std::string& signalName,
					     const std::string& jointName )
{
    createEndeffJacobianSignal(signalName, jointName);
}
void Dynamic::cmd_createPositionSignal( const std::string& signalName,
					const std::string& jointName )
{
    createPositionSignal(signalName, jointName);
}
