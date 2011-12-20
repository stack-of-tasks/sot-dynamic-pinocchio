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

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include "../src/dynamic-command.h"


using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Dynamic,"Dynamic");

using namespace std;

static matrix4d maalToMatrix4d(const ml::Matrix& inMatrix)
{
  matrix4d homogeneous;
  for (unsigned int r=0; r<4; r++) {
    for (unsigned int c=0; c<4; c++) {
      homogeneous(r,c) = inMatrix(r,c);
    }
  }
  return homogeneous;
}

static vector3d maalToVector3d(const ml::Vector& inVector)
{
  vector3d vector;
  vector(0) = inVector(0);
  vector(1) = inVector(1);
  vector(2) = inVector(2);
  return vector;
}

static matrix3d maalToMatrix3d(const ml::Matrix& inMatrix)
{
  matrix3d matrix;
  for (unsigned int r=0; r<3; r++) {
    for (unsigned int c=0; c<3; c++) {
      matrix(r,c) = inMatrix(r,c);
    }
  }
  return matrix;
}

Dynamic::
Dynamic( const std::string & name, bool build )
  :Entity(name)
  ,m_HDR(NULL)

  ,vrmlDirectory()
  ,vrmlMainFile()
  ,xmlSpecificityFile()
  ,xmlRankFile()

  ,init(false)

  ,jointPositionSIN(NULL,"sotDynamic("+name+")::input(vector)::position")
  ,freeFlyerPositionSIN(NULL,"sotDynamic("+name+")::input(vector)::ffposition")
  ,jointVelocitySIN(NULL,"sotDynamic("+name+")::input(vector)::velocity")
  ,freeFlyerVelocitySIN(NULL,"sotDynamic("+name+")::input(vector)::ffvelocity")
  ,jointAccelerationSIN(NULL,"sotDynamic("+name+")::input(vector)::acceleration")
  ,freeFlyerAccelerationSIN(NULL,"sotDynamic("+name+")::input(vector)::ffacceleration")

  ,firstSINTERN( boost::bind(&Dynamic::initNewtonEuler,this,_1,_2),
		 sotNOSIGNAL,"sotDynamic("+name+")::intern(dummy)::init" )
  ,newtonEulerSINTERN( boost::bind(&Dynamic::computeNewtonEuler,this,_1,_2),
		       firstSINTERN<<jointPositionSIN<<freeFlyerPositionSIN
		       <<jointVelocitySIN<<freeFlyerVelocitySIN
		       <<jointAccelerationSIN<<freeFlyerAccelerationSIN,
		       "sotDynamic("+name+")::intern(dummy)::newtoneuleur" )

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

  ,upperJlSOUT( boost::bind(&Dynamic::getUpperJointLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::upperJl" )

  ,lowerJlSOUT( boost::bind(&Dynamic::getLowerJointLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamic("+name+")::output(vector)::lowerJl" )

  ,inertiaRotorSOUT( "sotDynamic("+name+")::output(matrix)::inertiaRotor" )
  ,gearRatioSOUT( "sotDynamic("+name+")::output(matrix)::gearRatio" )
  ,inertiaRealSOUT( boost::bind(&Dynamic::computeInertiaReal,this,_1,_2),
		    inertiaSOUT << gearRatioSOUT << inertiaRotorSOUT,
		    "sotDynamic("+name+")::output(matrix)::inertiaReal" )
  ,MomentaSOUT( boost::bind(&Dynamic::computeMomenta,this,_1,_2),
		newtonEulerSINTERN,
		"sotDynamic("+name+")::output(vector)::momenta" )
  ,AngularMomentumSOUT( boost::bind(&Dynamic::computeAngularMomentum,this,_1,_2),
			newtonEulerSINTERN,
			"sotDynamic("+name+")::output(vector)::angularmomentum" )
  ,dynamicDriftSOUT( boost::bind(&Dynamic::computeTorqueDrift,this,_1,_2),
		     newtonEulerSINTERN,
		     "sotDynamic("+name+")::output(vector)::dynamicDrift" )
{
  sotDEBUGIN(5);

  if( build ) buildModel();

  firstSINTERN.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
  //DEBUG: Why =0? should be function. firstSINTERN.setConstant(0);

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
  signalRegistration(inertiaSOUT);
  signalRegistration(inertiaRealSOUT);
  signalRegistration(inertiaRotorSOUT);
  signalRegistration(gearRatioSOUT);
  signalRegistration( MomentaSOUT);
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
    "        - a string: directory containing main wrl file,\n"
    "        - a string: name of wrl file,\n"
    "        - a string: xml file containing humanoid specificities,\n"
    "        - a string: xml file defining order of joints in configuration"
    " vector.\n"
    "\n";
  addCommand("setFiles",
	     new command::SetFiles(*this, docstring));
  // parse
  docstring =
    "\n"
    "    Parse files to build an instance ot robot.\n"
    "\n"
    "      No input.\n"
    "      Files are defined by command setFiles \n"
    "\n";
    addCommand("parse",
	       new command::Parse(*this, docstring));

    {
      using namespace ::dynamicgraph::command;
      // CreateOpPoint
      docstring = "    \n"
	"    Create an operational point attached to a robot joint local frame.\n"
	"    \n"
	"      Input: \n"
	"        - a string: name of the operational point,\n"
	"        - a string: name the joint, among (gaze, left-ankle, right ankle\n"
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

    // SetProperty
    docstring = "    \n"
      "    Set a property.\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the property,\n"
      "        - a string: value of the property.\n"
      "    \n";
    addCommand("setProperty", new command::SetProperty(*this, docstring));

    docstring = "    \n"
      "    Get a property\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the property,\n"
      "      Return:\n"
      "        - a string: value of the property\n";
    addCommand("getProperty", new command::GetProperty(*this, docstring));

    docstring = "    \n"
      "    Create an empty robot\n"
      "    \n";
    addCommand("createRobot", new command::CreateRobot(*this, docstring));

    docstring = "    \n"
      "    Create a joint\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the joint,\n"
      "        - a string: type of the joint in ['freeflyer', 'rotation',\n"
      "                    'translation', 'anchor'],\n"
      "        - a matrix: (homogeneous) position of the joint.\n"
      "    \n";
    addCommand("createJoint", new command::CreateJoint(*this, docstring));

    docstring = "    \n"
      "    Set the root joint of the robot\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the joint.\n"
      "    \n";
    addCommand("setRootJoint", new command::SetRootJoint(*this, docstring));

    docstring = "    \n"
      "    Add a child joint to a joint\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the parent joint,\n"
      "        - a string: name of the child joint.\n"
      "    \n";
    addCommand("addJoint", new command::AddJoint(*this, docstring));

    docstring = "    \n"
      "    Set the bounds of a joint degree of freedom\n"
      "    \n"
      "      Input:\n"
      "        - a string: the name of the joint,\n"
      "        - a non-negative integer: the dof id in the joint\n"
      "        - a floating point number: the minimal value,\n"
      "        - a floating point number: the maximal value.\n"
      "    \n";
    addCommand("setDofBounds", new command::SetDofBounds(*this, docstring));

    docstring = "    \n"
      "    Set the mass of the body attached to a joint\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the joint,\n"
      "        - a floating point number: the mass of the body.\n"
      "    \n";
    addCommand("setMass", new command::SetMass(*this, docstring));

    docstring = "    \n"
      "    Set the position of the center of mass of a body\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the joint,\n"
      "        - a vector: the coordinate of the center of mass in the local\n"
      "                    frame of the joint.\n"
      "    \n";
    addCommand("setLocalCenterOfMass",
	       new command::SetLocalCenterOfMass(*this, docstring));

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
      "    Set specific joints of humanoid robot\n"
      "    \n"
      "      Input:\n"
      "        - a string: name of the joint,\n"
      "        - a string: type of the joint in ['waist', 'chest',"
      " 'left-wrist',\n"
      "                    'right-wrist', 'left-ankle', 'right-ankle',"
      " 'gaze']\n"
      "    \n";
    addCommand("setSpecificJoint",
	       new command::SetSpecificJoint(*this, docstring));

    docstring = "    \n"
      "    Set hand parameters\n"
      "    \n"
      "      Input:\n"
      "        - a boolean: whether right hand or not,\n"
      "        - a vector: the center of the hand in the wrist local frame,\n"
      "        - a vector: the thumb axis in the wrist local frame,\n"
      "        - a vector: the forefinger axis in the wrist local frame,\n"
      "        - a vector: the palm normal in the wrist local frame.\n"
      "    \n";
    addCommand("setHandParameters",
	       new command::SetHandParameters(*this, docstring));

    docstring = "    \n"
      "    Set foot parameters\n"
      "    \n"
      "      Input:\n"
      "        - a boolean: whether right foot or not,\n"
      "        - a floating point number: the sole length,\n"
      "        - a floating point number: the sole width,\n"
      "        - a vector: the position of the ankle in the foot local frame.\n"
      "    \n";
    addCommand("setFootParameters",
	       new command::SetFootParameters(*this, docstring));

    docstring = "    \n"
      "    Set parameters of the gaze\n"
      "    \n"
      "      Input:\n"
      "        - a vector: the gaze origin,\n"
      "        - a vector: the gaze direction.\n"
      "    \n";
    addCommand("setGazeParameters",
	       new command::SetGazeParameters(*this, docstring));

    docstring = "    \n"
      "    Initialize kinematic chain of robot\n"
      "    \n";
    addCommand("initializeRobot",
	       new command::InitializeRobot(*this, docstring));

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

    docstring = "    \n"
      "    Get left foot sole length.\n"
      "    \n";
    addCommand("getSoleLength",
	       new dynamicgraph::command::Getter<Dynamic, double>
	       (*this, &Dynamic::getSoleLength, docstring));
    docstring = "    \n"
      "    Get left foot sole width.\n"
      "    \n";
    addCommand("getSoleWidth",
	       new dynamicgraph::command::Getter<Dynamic, double>
	       (*this, &Dynamic::getSoleWidth, docstring));

    docstring = "    \n"
      "    Get ankle position in left foot frame.\n"
      "    \n";
    addCommand("getAnklePositionInFootFrame",
	       new dynamicgraph::command::Getter<Dynamic, ml::Vector>
	       (*this, &Dynamic::getAnklePositionInFootFrame, docstring));

  sotDEBUGOUT(5);
}

void Dynamic::
buildModel( void )
{
  sotDEBUGIN(5);

  djj::ObjectFactory aRobotDynamicsObjectConstructor;

  m_HDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();

  sotDEBUGOUT(5);
}


Dynamic::
~Dynamic( void )
{
  sotDEBUGIN(5);
  if( 0!=m_HDR )
    {
      delete m_HDR;
      m_HDR = 0;
    }

  for(  std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
	iter != genericSignalRefs.end();
	++iter )
    {
      SignalBase<int>* sigPtr = *iter;
      delete sigPtr;
    }

  sotDEBUGOUT(5);
  return;
}

/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
/* --- CONFIG --------------------------------------------------------------- */
void Dynamic::
setVrmlDirectory( const std::string& filename )
{
  vrmlDirectory = filename;
}
void Dynamic::
setVrmlMainFile( const std::string& filename )
{
  vrmlMainFile = filename;
}
void Dynamic::
setXmlSpecificityFile( const std::string& filename )
{
  xmlSpecificityFile = filename;
}
void Dynamic::
setXmlRankFile( const std::string& filename )
{
  xmlRankFile = filename;
}


// Helper macro for Dynamic::parseConfigFiles().
// Check that all required files exist or throw an exception
// otherwise.
#define CHECK_FILE(PATH, FILE_DESCRIPTION)				\
  do									\
    {									\
      if (!boost::filesystem::exists (PATH)				\
	  || boost::filesystem::is_directory (PATH))			\
	{								\
	  boost::format fmt ("Failed to open the %s (%s).");		\
	  fmt % (FILE_DESCRIPTION) % robotModelPath.file_string ();	\
	  								\
	  SOT_THROW ExceptionDynamic					\
	    (ExceptionDynamic::DYNAMIC_JRL,				\
	     fmt.str ());						\
	}								\
    }									\
  while (0)

void Dynamic::parseConfigFiles()
{
  sotDEBUGIN(15);

  // Construct the full path to the robot model.
  boost::filesystem::path robotModelPath (vrmlDirectory);
  robotModelPath /= vrmlMainFile;

  boost::filesystem::path xmlRankPath (xmlRankFile);
  boost::filesystem::path xmlSpecificityPath (xmlSpecificityFile);

  CHECK_FILE (robotModelPath, "HRP-2 model");
  CHECK_FILE (xmlRankPath, "XML rank file");
  CHECK_FILE (xmlSpecificityFile, "XML specificity file");

  try
    {
      sotDEBUG(35) << "Parse the vrml."<<endl;

      std::string robotModelPathStr (robotModelPath.file_string());
      std::string xmlRankPathStr (xmlRankPath.file_string());
      std::string xmlSpecificityPathStr (xmlSpecificityPath.file_string());

      djj::parseOpenHRPVRMLFile (*m_HDR,
				 robotModelPathStr,
				 xmlRankPathStr,
				 xmlSpecificityPathStr);
    }
  catch (...)
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::DYNAMIC_JRL,
				  "Error while parsing." );
    }

  init = true;
  sotDEBUGOUT(15);
}

#undef CHECK_FILE

/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
dg::SignalTimeDependent< ml::Matrix,int > & Dynamic::
createJacobianSignal( const std::string& signame, CjrlJoint* aJoint )
{
  dg::SignalTimeDependent< ml::Matrix,int > * sig
    = new dg::SignalTimeDependent< ml::Matrix,int >
    ( boost::bind(&Dynamic::computeGenericJacobian,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrix)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  return *sig;
}

dg::SignalTimeDependent< ml::Matrix,int > & Dynamic::
createEndeffJacobianSignal( const std::string& signame, CjrlJoint* aJoint )
{
  sotDEBUGIN(15);

  dg::SignalTimeDependent< ml::Matrix,int > * sig
    = new dg::SignalTimeDependent< ml::Matrix,int >
    ( boost::bind(&Dynamic::computeGenericEndeffJacobian,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrix)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}

void Dynamic::
destroyJacobianSignal( const std::string& signame )
{
  bool deletable = false;
  dg::SignalTimeDependent< ml::Matrix,int > * sig = & jacobiansSOUT( signame );
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
				     " (while trying to remove generic jac. signal <%s>).",
				     signame.c_str() );
    }

  signalDeregistration( signame );

  delete sig;
}

/* --- POINT --- */
/* --- POINT --- */
/* --- POINT --- */

dg::SignalTimeDependent< MatrixHomogeneous,int >& Dynamic::
createPositionSignal( const std::string& signame, CjrlJoint* aJoint)
{
  sotDEBUGIN(15);

  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig
    = new dg::SignalTimeDependent< MatrixHomogeneous,int >
    ( boost::bind(&Dynamic::computeGenericPosition,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrixHomo)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}


void Dynamic::
destroyPositionSignal( const std::string& signame )
{
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

/* --- VELOCITY --- */
/* --- VELOCITY --- */
/* --- VELOCITY --- */

SignalTimeDependent< ml::Vector,int >& Dynamic::
createVelocitySignal( const std::string& signame, CjrlJoint* aJoint )
{
  sotDEBUGIN(15);
  SignalTimeDependent< ml::Vector,int > * sig
    = new SignalTimeDependent< ml::Vector,int >
    ( boost::bind(&Dynamic::computeGenericVelocity,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(ml::Vector)::"+signame );
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}


void Dynamic::
destroyVelocitySignal( const std::string& signame )
{
  bool deletable = false;
  SignalTimeDependent< ml::Vector,int > * sig = & velocitiesSOUT( signame );
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

/* --- ACCELERATION --- */
/* --- ACCELERATION --- */
/* --- ACCELERATION --- */

dg::SignalTimeDependent< ml::Vector,int >& Dynamic::
createAccelerationSignal( const std::string& signame, CjrlJoint* aJoint )
{
  sotDEBUGIN(15);
  dg::SignalTimeDependent< ml::Vector,int > * sig
    = new dg::SignalTimeDependent< ml::Vector,int >
    ( boost::bind(&Dynamic::computeGenericAcceleration,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrixHomo)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}


void Dynamic::
destroyAccelerationSignal( const std::string& signame )
{
  bool deletable = false;
  dg::SignalTimeDependent< ml::Vector,int > * sig = & accelerationsSOUT( signame );
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
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */
/* --- COMPUTE -------------------------------------------------------------- */

#include <jrl/mal/boostspecific.hh>

static void MAAL1_V3d_to_MAAL2( const vector3d& source,
				ml::Vector & res )
{
  sotDEBUG(5) << source <<endl;
  res(0) = source[0];
  res(1) = source[1];
  res(2) = source[2];
}

ml::Matrix& Dynamic::
computeGenericJacobian( CjrlJoint * aJoint,ml::Matrix& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);

  aJoint->computeJacobianJointWrtConfig();
  res.initFromMotherLib(aJoint->jacobianJointWrtConfig());
  sotDEBUGOUT(25);

  return res;
}

ml::Matrix& Dynamic::
computeGenericEndeffJacobian( CjrlJoint * aJoint,ml::Matrix& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);

  aJoint->computeJacobianJointWrtConfig();

  ml::Matrix J,V(6,6);
  J.initFromMotherLib(aJoint->jacobianJointWrtConfig());

  /* --- TODO --- */
  MatrixHomogeneous M;
  computeGenericPosition(aJoint,M,time);
  //M=M.inverse();

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	V(i,j)=M(j,i);
	V(i+3,j+3)=M(j,i);
	V(i+3,j)=0.;
	V(i,j+3)=0.;
      }

  sotDEBUG(25) << "0Jn = "<< J;
  sotDEBUG(25) << "V = "<< V;
  V.multiply(J,res);
  sotDEBUGOUT(25);

  return res;
}

MatrixHomogeneous& Dynamic::
computeGenericPosition( CjrlJoint * aJoint,MatrixHomogeneous& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  const matrix4d & m4 = aJoint->currentTransformation();

  res.resize(4,4);
  for( int i=0;i<4;++i )
    for( int j=0;j<4;++j )
      res(i,j) = MAL_S4x4_MATRIX_ACCESS_I_J(m4,i,j);

  //  aJoint->computeJacobianJointWrtConfig();
  //res.initFromMotherLib(aJoint->jacobianJointWrtConfig());

  // adaptation to the new dynamic -- to be optimized
    matrix4d initialTr;
    initialTr = aJoint->initialPosition();
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,0,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,1,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,2,3) = 0.0;

    matrix4d invrot;
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<3;j++)
        {
  	MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j)=0.0;
  	for(unsigned int k=0;k<3;k++)
  	  {
  	    MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j)+=
  	      MAL_S4x4_MATRIX_ACCESS_I_J(res,i,k) *
  	      MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,j,k);
  	  }
        }
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<3;j++)
        MAL_S4x4_MATRIX_ACCESS_I_J(res,i,j) =
  	MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j);
    //end of the adaptation


  sotDEBUGOUT(25);
  return res;
}

ml::Vector& Dynamic::
computeGenericVelocity( CjrlJoint* j,ml::Vector& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  CjrlRigidVelocity aRV = j->jointVelocity();
  vector3d al= aRV.linearVelocity();
  vector3d ar= aRV.rotationVelocity();

  res.resize(6);
  for( int i=0;i<3;++i )
    {
      res(i)=al(i);
      res(i+3)=ar(i);
    }

  sotDEBUGOUT(25);
  return res;
}

ml::Vector& Dynamic::
computeGenericAcceleration( CjrlJoint* j,ml::Vector& res,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  CjrlRigidAcceleration aRA = j->jointAcceleration();
  vector3d al= aRA.linearAcceleration();
  vector3d ar= aRA.rotationAcceleration(); // TODO: Dont copy, reference.

  res.resize(6);
  for( int i=0;i<3;++i )
    {
      res(i)=al(i);
      res(i+3)=ar(i);
    }

  sotDEBUGOUT(25);
  return res;
}



ml::Vector& Dynamic::
computeZmp( ml::Vector& ZMPval,int time )
{
  if (ZMPval.size()!=3)
    ZMPval.resize(3);

  newtonEulerSINTERN(time);
  MAAL1_V3d_to_MAAL2(m_HDR->zeroMomentumPoint(),ZMPval);
  sotDEBUGOUT(25);
  return ZMPval;
}


ml::Vector& Dynamic::
computeMomenta(ml::Vector & Momenta, int time)
{
  vector3d LinearMomentum, AngularMomentum;

  if (Momenta.size()!=6)
    Momenta.resize(6);

  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  LinearMomentum = m_HDR->linearMomentumRobot();
  AngularMomentum = m_HDR->angularMomentumRobot();

  for(unsigned int i=0;i<3;i++)
    {
      Momenta(i)   = LinearMomentum(i);
      Momenta(i+3) = AngularMomentum(i);
    }

  sotDEBUGOUT(25) << "Momenta :" << Momenta ;
  return Momenta;
}

ml::Vector& Dynamic::
computeAngularMomentum(ml::Vector & Momenta, int time)
{
  vector3d  AngularMomentum;

  if (Momenta.size()!=3)
    Momenta.resize(3);

  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  AngularMomentum = m_HDR->angularMomentumRobot();

  for(unsigned int i=0;i<3;i++)
    {
      Momenta(i) = AngularMomentum(i);
    }

  sotDEBUGOUT(25) << "AngularMomenta :" << Momenta ;
  return Momenta;

}

ml::Matrix& Dynamic::
computeJcom( ml::Matrix& Jcom,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);

  matrixNxP jacobian;
  jacobian.resize(3, m_HDR->numberDof());
  m_HDR->getJacobianCenterOfMass(*m_HDR->rootJoint(), jacobian);

  Jcom.initFromMotherLib(jacobian);
  sotDEBUGOUT(25);
  return Jcom;
}

ml::Vector& Dynamic::
computeCom( ml::Vector& com,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  com.resize(3);
  MAAL1_V3d_to_MAAL2(m_HDR->positionCenterOfMass(),com);
  sotDEBUGOUT(25);
  return com;
}

ml::Matrix& Dynamic::
computeInertia( ml::Matrix& A,int time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);

  m_HDR->computeInertiaMatrix();
  A.initFromMotherLib(m_HDR->inertiaMatrix());

  if( 1==debugInertia )
    {
      for( unsigned int i=0;i<18;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
      for( unsigned int i=20;i<22;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
      for( unsigned int i=28;i<36;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
    }
  else if( 2==debugInertia )
    {
      for( unsigned int i=0;i<18;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
      for( unsigned int i=20;i<22;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
      for( unsigned int i=28;i<29;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
      for( unsigned int i=35;i<36;++i )
	for( unsigned int j=0;j<36;++j )
	  if( i==j ) A(i,i)=1;
	  else {  A(i,j)=A(j,i)=0; }
    }

  sotDEBUGOUT(25);
  return A;
}

ml::Matrix& Dynamic::
computeInertiaReal( ml::Matrix& res,int time )
{
  sotDEBUGIN(25);

  const ml::Matrix & A = inertiaSOUT(time);
  const ml::Vector & gearRatio = gearRatioSOUT(time);
  const ml::Vector & inertiaRotor = inertiaRotorSOUT(time);

  res = A;
  for( unsigned int i=0;i<gearRatio.size();++i )
    res(i,i) += (gearRatio(i)*gearRatio(i)*inertiaRotor(i));

  sotDEBUGOUT(25);
  return res;
}

double& Dynamic::
computeFootHeight (double&, int time)
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  CjrlFoot* RightFoot = m_HDR->rightFoot();
  vector3d AnkleInLocalRefFrame;
  RightFoot->getAnklePositionInLocalFrame(AnkleInLocalRefFrame);
  sotDEBUGOUT(25);
  return AnkleInLocalRefFrame[2];
}


/* --- SIGNAL --------------------------------------------------------------- */
/* --- SIGNAL --------------------------------------------------------------- */
/* --- SIGNAL --------------------------------------------------------------- */

dg::SignalTimeDependent<ml::Matrix,int>& Dynamic::
jacobiansSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);

  try {
    dg::SignalTimeDependent<ml::Matrix,int>& res
      = dynamic_cast< dg::SignalTimeDependent<ml::Matrix,int>& >( sigabs );
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

dg::SignalTimeDependent<ml::Vector,int>& Dynamic::
velocitiesSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);
  try {
    dg::SignalTimeDependent<ml::Vector,int>& res
      = dynamic_cast< dg::SignalTimeDependent<ml::Vector,int>& >( sigabs );
    return res;
 } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type Vector.",
				  name.c_str());
  }
}

dg::SignalTimeDependent<ml::Vector,int>& Dynamic::
accelerationsSOUT( const std::string& name )
{
  SignalBase<int> & sigabs = Entity::getSignal(name);

  try {
    dg::SignalTimeDependent<ml::Vector,int>& res
      = dynamic_cast< dg::SignalTimeDependent<ml::Vector,int>& >( sigabs );
    return res;
  } catch( std::bad_cast e ) {
    SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
				  "Impossible cast.",
				  " (while getting signal <%s> of type Vector.",
				  name.c_str());
  }
}


int& Dynamic::
computeNewtonEuler( int& dummy,int time )
{
  sotDEBUGIN(15);
  ml::Vector pos = jointPositionSIN(time); // TODO &pos
  ml::Vector vel = jointVelocitySIN(time);
  ml::Vector acc = jointAccelerationSIN(time);

  sotDEBUG(5) << "computeNewtonEuler: " << pos << endl;
  firstSINTERN(time);
  if( freeFlyerPositionSIN )
    {
      const ml::Vector& ffpos = freeFlyerPositionSIN(time);

      for( int i=0;i<6;++i ) pos(i) = ffpos(i) ;
      sotDEBUG(5) << "computeNewtonEuler: (" << name << ") ffpos = " << ffpos << endl;
   }
  sotDEBUG(5) << "computeNewtonEuler: (" << name << ") pos = " << pos << endl;
  if( freeFlyerVelocitySIN )
    {
      const ml::Vector& ffvel = freeFlyerVelocitySIN(time);
      for( int i=0;i<6;++i ) vel(i) = ffvel(i);
    }
  if( freeFlyerAccelerationSIN )
    {
      const ml::Vector& ffacc = freeFlyerAccelerationSIN(time);
      for( int i=0;i<6;++i ) acc(i) = ffacc(i);
    }
  if(! m_HDR->currentConfiguration(pos.accessToMotherLib()))
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_SIZE,
				  getName() +
				  ": position vector size incorrect",
				  " (Vector size is %d, should be %d).",
				  pos.size(),
				  m_HDR->currentConfiguration().size() );
    }


  if(! m_HDR->currentVelocity(vel.accessToMotherLib()) )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_SIZE,
				  getName() +
				  ": velocity vector size incorrect",
				  " (Vector size is %d, should be %d).",
				  vel.size(),
				  m_HDR->currentVelocity().size() );
    }

  if(! m_HDR->currentAcceleration(acc.accessToMotherLib()) )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_SIZE,
				  getName() +
				  ": acceleration vector size incorrect",
				  " (Vector size is %d, should be %d).",
				  acc.size(),
				  m_HDR->currentAcceleration().size() );
    }

  m_HDR->computeForwardKinematics();

  sotDEBUG(1)<< "pos = " <<pos <<endl;
  sotDEBUG(1)<< "vel = " <<vel <<endl;
  sotDEBUG(1)<< "acc = " <<acc <<endl;

  sotDEBUGOUT(15);
  return dummy;
}
int& Dynamic::
initNewtonEuler( int& dummy,int time )
{
  sotDEBUGIN(15);
  firstSINTERN.setReady(false);
  computeNewtonEuler(dummy,time);
  for( int i=0;i<3;++i )
    m_HDR->computeForwardKinematics();

  sotDEBUGOUT(15);
  return dummy;
}

ml::Vector& Dynamic::
getUpperJointLimits(ml::Vector& res, const int&)
{
  sotDEBUGIN(15);
  const unsigned int NBJ = m_HDR->numberDof();
  res.resize( NBJ );
  for( unsigned int i=0;i<NBJ;++i )
    {
      res(i)=m_HDR->upperBoundDof( i );
    }
  sotDEBUG(15) << "upperLimit (" << NBJ << ")=" << res <<endl;
  sotDEBUGOUT(15);
  return res;
}

ml::Vector& Dynamic::
getLowerJointLimits(ml::Vector& res, const int&)
{
  sotDEBUGIN(15);
  const unsigned int NBJ = m_HDR->numberDof();
  res.resize( NBJ );
  for( unsigned int i=0;i<NBJ;++i )
    {
      res(i)=m_HDR->lowerBoundDof( i );
    }
  sotDEBUG(15) << "lowerLimit (" << NBJ << ")=" << res <<endl;
  sotDEBUGOUT(15);
  return res;
}

ml::Vector& Dynamic::
computeTorqueDrift( ml::Vector& tauDrift,const int  & iter )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(iter);
  const unsigned int NB_JOINTS = jointPositionSIN.accessCopy().size();

  tauDrift.resize(NB_JOINTS);
  const vectorN& Torques = m_HDR->currentJointTorques();
  for( unsigned int i=0;i<NB_JOINTS; ++i ) tauDrift(i) = Torques(i);

  sotDEBUGOUT(25);
  return tauDrift;
}

/* --- COMMANDS ------------------------------------------------------------- */
/* --- COMMANDS ------------------------------------------------------------- */
/* --- COMMANDS ------------------------------------------------------------- */

CjrlJoint* Dynamic::getJointByName( const std::string& jointName )
{
  if (jointName ==  "gaze") {
    return m_HDR->gazeJoint();
  } else if (jointName == "left-ankle") {
    return m_HDR->leftAnkle();
  } else if (jointName == "right-ankle") {
    return m_HDR->rightAnkle();
  } else if (jointName == "left-wrist") {
    return m_HDR->leftWrist();
  } else if (jointName == "right-wrist") {
    return m_HDR->rightWrist();
  } else if (jointName == "waist") {
    return m_HDR->waist();
  } else if (jointName == "chest") {
    return m_HDR->chest();
  } else if (jointName == "gaze") {
    return m_HDR->gazeJoint();
  } 
  
  // left toe
  else if (jointName == "left-toe") 
  {
    if (m_HDR->leftAnkle()->countChildJoints () == 0)
    	throw ExceptionDynamic(ExceptionDynamic::GENERIC," The robot has no toes");
    else
		return m_HDR->leftAnkle()->childJoint(0);
  } 

  // right toe
  else if (jointName == "right-toe") {
    if (m_HDR->rightAnkle()->countChildJoints () == 0)
    	throw ExceptionDynamic(ExceptionDynamic::GENERIC," The robot has no toes");
    else
		return m_HDR->rightAnkle()->childJoint(0);

  }
  else {
    std::vector< CjrlJoint* > jv = m_HDR->jointVector ();
    for (std::vector< CjrlJoint* >::const_iterator it = jv.begin();
	 it != jv.end(); it++) {
      if ((*it)->getName () == jointName) {
	return *it;
      }
    }
  }
  throw ExceptionDynamic(ExceptionDynamic::GENERIC,
			 jointName + " is not a valid name."
			 " Valid names are \n"
			 "gaze, left-ankle, right-ankle, left-wrist,"
			 " right-wrist, waist, chest, or any joint name.");
}

void Dynamic::cmd_createOpPointSignals( const std::string& opPointName,
				 const std::string& jointName )
{
  CjrlJoint* joint = getJointByName(jointName);
  createEndeffJacobianSignal(std::string("J")+opPointName, joint);
  createPositionSignal(opPointName, joint);
}
void Dynamic::cmd_createJacobianWorldSignal( const std::string& signalName,
				 const std::string& jointName )
{
  CjrlJoint* joint = getJointByName(jointName);
  createJacobianSignal(signalName, joint);
}
void Dynamic::cmd_createJacobianEndEffectorSignal( const std::string& signalName,
					     const std::string& jointName )
{
  CjrlJoint* joint = getJointByName(jointName);
  createEndeffJacobianSignal(signalName, joint);
}
void Dynamic::cmd_createPositionSignal( const std::string& signalName,
					const std::string& jointName )
{
  CjrlJoint* joint = getJointByName(jointName);
  createPositionSignal(signalName, joint);
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
void Dynamic::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "# In { Cmd " << cmdLine <<endl;
  std::string filename;
  if( cmdLine == "debugInertia" )
    {
      cmdArgs>>ws; if(cmdArgs.good())
		     {
		       std::string arg; cmdArgs >> arg;
		       if( (arg=="true")||(arg=="1") )
			 { debugInertia = 1; }
		       else if( (arg=="2")||(arg=="grip") )
			 { debugInertia = 2; }
		       else debugInertia=0;

		     }
      else { os << "debugInertia = " << debugInertia << std::endl; }
    }
  else if( cmdLine == "setVrmlDir" )
    {  cmdArgs>>filename; setVrmlDirectory( filename );  }
  else if( cmdLine == "setVrml" )
    {  cmdArgs>>filename; setVrmlMainFile( filename );  }
  else if( cmdLine == "setXmlSpec" )
    {  cmdArgs>>filename; setXmlSpecificityFile( filename );  }
  else if( cmdLine == "setXmlRank" )
    {  cmdArgs>>filename; setXmlRankFile( filename );  }
  else if( cmdLine == "setFiles" )
    {
      cmdArgs>>filename; setVrmlDirectory( filename );
      cmdArgs>>filename; setVrmlMainFile( filename );
      cmdArgs>>filename; setXmlSpecificityFile( filename );
      cmdArgs>>filename; setXmlRankFile( filename );
    }
  else if( cmdLine == "displayFiles" )
    {
      cmdArgs >> ws; bool filespecified = false;
      if( cmdArgs.good() )
	{
	  filespecified = true;
	  std::string filetype; cmdArgs >> filetype;
	  sotDEBUG(15) << " Request: " << filetype << std::endl;
	  if( "vrmldir" == filetype ) { os << vrmlDirectory << std::endl; }
	  else if( "xmlspecificity" == filetype ) { os << xmlSpecificityFile << std::endl; }
	  else if( "xmlrank" == filetype ) { os << xmlRankFile << std::endl; }
	  else if( "vrmlmain" == filetype ) { os << vrmlMainFile << std::endl; }
	  else filespecified = false;
	}
      if( ! filespecified )
	{
	  os << "  - VRML Directory:\t\t\t" << vrmlDirectory <<endl
	     << "  - XML Specificity File:\t\t" << xmlSpecificityFile <<endl
	     << "  - XML Rank File:\t\t\t" << xmlRankFile <<endl
	     << "  - VRML Main File:\t\t\t" << vrmlMainFile <<endl;
	}
    }
  else if( cmdLine == "parse" )
    {
      if(! init )parseConfigFiles(); else cout << "  !! Already parsed." <<endl;
    }
  else if( cmdLine == "createJacobian" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createJacobianSignal(Jname,rank);
    }
  else if( cmdLine == "destroyJacobian" )
    {
      std::string Jname; cmdArgs >> Jname;
      destroyJacobianSignal(Jname);
    }
  else if( cmdLine == "createPosition" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createPositionSignal(Jname,rank);
    }
  else if( cmdLine == "destroyPosition" )
    {
      std::string Jname; cmdArgs >> Jname;
      destroyPositionSignal(Jname);
    }
  else if( cmdLine == "createVelocity" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createVelocitySignal(Jname,rank);
    }
  else if( cmdLine == "destroyVelocity" )
    {
      std::string Jname; cmdArgs >> Jname;
      destroyVelocitySignal(Jname);
    }
  else if( cmdLine == "createAcceleration" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createAccelerationSignal(Jname,rank);
    }
  else if( cmdLine == "destroyAcceleration" )
    {
      std::string Jname; cmdArgs >> Jname;
      destroyAccelerationSignal(Jname);
    }
  else if( cmdLine == "createEndeffJacobian" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createEndeffJacobianSignal(Jname,rank);
    }
  else if( cmdLine == "createOpPoint" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      //createEndeffJacobianSignal(string("J")+Jname,rank);
      //createPositionSignal(Jname,rank);
      sotDEBUG(15)<<endl;
    }
  else if( cmdLine == "destroyOpPoint" )
    {
      std::string Jname; cmdArgs >> Jname;
      destroyJacobianSignal(string("J")+Jname);
      destroyPositionSignal(Jname);
    }
  else if( cmdLine == "ndof" ) { os << m_HDR->numberDof() <<endl; return; }
  else if( cmdLine == "setComputeCom" )
    {
      unsigned int b; cmdArgs >> b;
      comActivation((b!=0));
    }
  else if( cmdLine == "setComputeZmp" )
    {
      unsigned int b; cmdArgs >> b;
      zmpActivation((b!=0));
    }
  else if( cmdLine == "setProperty" )
    {
      string prop,val; cmdArgs >> prop;
      if( cmdArgs.good() ) cmdArgs >> val; else val="true";
      m_HDR->setProperty( prop,val );
    }
  else if( cmdLine == "getProperty" )
    {
      string prop,val; cmdArgs >> prop;
      m_HDR->getProperty( prop,val );
      os<<val<<endl;
    }
  else if( cmdLine == "displayProperties" )
    {
      std::istringstream iss("ComputeVelocity ComputeCoM ComputeAccelerationCoM ComputeMomentum ComputeZMP ComputeBackwardDynamics");
      string prop,val; const unsigned int STR_SIZE=30;
      while( iss.good() )
	{
	  iss>>prop;
	  m_HDR->getProperty( prop,val );
	  os<<prop;
	  for( unsigned int i=prop.length();i<STR_SIZE;++i ) os<<" ";
	  os<<" -> "<<val <<endl;
	}
    }
  else if( cmdLine == "help" )
    {
      os << "Dynamics:"<<endl
	 << "  - setVrmlDir - setVrml - setXmlSpec - setXmlRanks <file>" <<endl
	 << "\t\t\t\t:set the config files" <<endl
	 << "  - setFiles <%1> ... <%4>\t:set files in the order cited above" <<endl
	 << "  - displayFiles\t\t\t:display the 5 config files" <<endl
	 << "  - parse\t\t\t:parse the files set unsing the set{Xml|Vrml} commands." <<endl
	 << "  - createJacobian <name> <point>:create a signal named <name> " << endl
	 << "  - createEndeffJacobian <name> <point>:create a signal named <name> "
	 << "forwarding the jacoian computed at <point>." <<endl
	 << "  - destroyJacobian <name>\t:delete the jacobian signal <name>" << endl
	 << "  - {create|destroy}Position\t:handle position signals." <<endl
	 << "  - {create|destroy}OpPoint\t:handle Operation Point (ie pos+jac) signals." <<endl
	 << "  - {create|destroy}Acceleration\t:handle acceleration signals." <<endl
	 << "  - {get|set}Property <name> [<val>]: set/get the property." <<endl
	 << "  - displayProperties: print the prop-val couples list." <<endl
	 << "  - ndof\t\t\t: display the number of DOF of the robot."<< endl;

      Entity::commandLine(cmdLine,cmdArgs,os);
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }

  sotDEBUGOUT(15);

}

void Dynamic::createRobot()
{
  if (m_HDR)
    delete m_HDR;
  m_HDR = factory_.createHumanoidDynamicRobot();
}

void Dynamic::createJoint(const std::string& inJointName,
			  const std::string& inJointType,
			   const ml::Matrix& inPosition)
{
  if (jointMap_.count(inJointName) == 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "a joint with name " + inJointName +
			       " has already been created.");
  }
  matrix4d position = maalToMatrix4d(inPosition);
  CjrlJoint* joint=NULL;

  if (inJointType == "freeflyer") {
    joint = factory_.createJointFreeflyer(position);
  } else if (inJointType == "rotation") {
    joint = factory_.createJointRotation(position);
  } else if (inJointType == "translation") {
    joint = factory_.createJointTranslation(position);
  } else if (inJointType == "anchor") {
    joint = factory_.createJointAnchor(position);
  } else {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       inJointType + " is not a valid type.\n"
			       "Valid types are 'freeflyer', 'rotation', 'translation', 'anchor'.");
  }
  jointMap_[inJointName] = joint;
}

void Dynamic::setRootJoint(const std::string& inJointName)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  m_HDR->rootJoint(*jointMap_[inJointName]);
}

void Dynamic::addJoint(const std::string& inParentName,
		       const std::string& inChildName)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inParentName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inParentName +
			       " has been created.");
  }
  if (jointMap_.count(inChildName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inChildName +
			       " has been created.");
  }
  jointMap_[inParentName]->addChildJoint(*(jointMap_[inChildName]));
}

void Dynamic::setDofBounds(const std::string& inJointName,
			   unsigned int inDofId,
			   double inMinValue, double inMaxValue)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  jointMap_[inJointName]->lowerBound(inDofId, inMinValue);
  jointMap_[inJointName]->upperBound(inDofId, inMaxValue);
}

void Dynamic::setMass(const std::string& inJointName, double inMass)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  CjrlJoint* joint = jointMap_[inJointName];
  if (!joint->linkedBody()) {
    joint->setLinkedBody(*(factory_.createBody()));
  }
  CjrlBody& body = *(joint->linkedBody());
  body.mass(inMass);
}

void Dynamic::setLocalCenterOfMass(const std::string& inJointName,
				   ml::Vector inCom)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  CjrlJoint* joint = jointMap_[inJointName];
  if (!joint->linkedBody()) {
    joint->setLinkedBody(*(factory_.createBody()));
  }
  CjrlBody& body = *(joint->linkedBody());
  body.localCenterOfMass(maalToVector3d(inCom));
}

void Dynamic::setInertiaMatrix(const std::string& inJointName,
			       ml::Matrix inMatrix)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  CjrlJoint* joint = jointMap_[inJointName];
  if (!joint->linkedBody()) {
    joint->setLinkedBody(*(factory_.createBody()));
  }
  CjrlBody& body = *(joint->linkedBody());
  body.inertiaMatrix(maalToMatrix3d(inMatrix));
}

void Dynamic::setSpecificJoint(const std::string& inJointName,
			       const std::string& inJointType)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  if (jointMap_.count(inJointName) != 1) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "No joint with name " + inJointName +
			       " has been created.");
  }
  CjrlJoint* joint = jointMap_[inJointName];
  if (inJointType == "chest") {
  } else if (inJointType == "waist") {
    m_HDR->waist(joint);
  } else if (inJointType == "chest") {
    m_HDR->chest(joint);
  } else if (inJointType == "left-wrist") {
    m_HDR->leftWrist(joint);
    // Create hand
    CjrlHand* hand = factory_.createHand(joint);
    m_HDR->leftHand(hand);
  } else if (inJointType == "right-wrist") {
    m_HDR->rightWrist(joint);
    // Create hand
    CjrlHand* hand = factory_.createHand(joint);
    m_HDR->rightHand(hand);
  } else if (inJointType == "left-ankle") {
    m_HDR->leftAnkle(joint);
    // Create foot
    CjrlFoot* foot = factory_.createFoot(joint);
    m_HDR->leftFoot(foot);
  } else if (inJointType == "right-ankle") {
    m_HDR->rightAnkle(joint);
    // Create foot
    CjrlFoot* foot = factory_.createFoot(joint);
    m_HDR->rightFoot(foot);
  } else if (inJointType == "gaze") {
    m_HDR->gazeJoint(joint);
  } else {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       inJointType + " is not a valid type.\n"
			       "Valid types are 'waist', 'chest', 'left-wrist',\n'right-wrist', 'left-ankle', 'right-ankle', 'gaze'.");
  }
}

void Dynamic::setHandParameters(bool inRight, const ml::Vector& inCenter,
				const ml::Vector& inThumbAxis,
				const ml::Vector& inForefingerAxis,
				const ml::Vector& inPalmNormal)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  CjrlHand *hand = NULL;
  if (inRight) {
    hand = m_HDR->rightHand();
  } else {
    hand = m_HDR->leftHand();
  }
  if (!hand) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "hand has not been defined yet");
  }
  hand->setCenter(maalToVector3d(inCenter));
  hand->setThumbAxis(maalToVector3d(inThumbAxis));
  hand->setForeFingerAxis(maalToVector3d(inForefingerAxis));
  hand->setPalmNormal(maalToVector3d(inPalmNormal));
}

void Dynamic::setFootParameters(bool inRight, const double& inSoleLength,
				const double& inSoleWidth,
				const ml::Vector& inAnklePosition)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  CjrlFoot *foot = NULL;
  if (inRight) {
    foot = m_HDR->rightFoot();
  } else {
    foot = m_HDR->leftFoot();
  }
  if (!foot) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "foot has not been defined yet");
  }
  foot->setSoleSize(inSoleLength, inSoleWidth);
  foot->setAnklePositionInLocalFrame(maalToVector3d(inAnklePosition));
}

double Dynamic::getSoleLength() const
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  CjrlFoot *foot = m_HDR->leftFoot();
  if (!foot) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "left foot has not been defined yet");
  }
  double length, width;
  foot->getSoleSize(length, width);
  return length;
}

double Dynamic::getSoleWidth() const
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  CjrlFoot *foot = m_HDR->leftFoot();
  if (!foot) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "left foot has not been defined yet");
  }
  double length, width;
  foot->getSoleSize(length, width);
  return width;
}

ml::Vector Dynamic::getAnklePositionInFootFrame() const
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  CjrlFoot *foot = m_HDR->leftFoot();
  if (!foot) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "left foot has not been defined yet");
  }
  vector3d anklePosition;
  foot->getAnklePositionInLocalFrame(anklePosition);
  ml::Vector res(3);
  res(0) = anklePosition[0];
  res(1) = anklePosition[1];
  res(2) = anklePosition[2];
  return res;
}

void Dynamic::setGazeParameters(const ml::Vector& inGazeOrigin,
				const ml::Vector& inGazeDirection)
{
  if (!m_HDR) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::DYNAMIC_JRL,
			       "you must create a robot first.");
  }
  m_HDR->gaze(maalToVector3d(inGazeDirection),
	      maalToVector3d(inGazeOrigin));
}

std::ostream& sot::operator<<(std::ostream& os,
			      const CjrlHumanoidDynamicRobot&)
{
  /*
  os << "Device: " << &robot << std::endl;
  os << std::endl;
  os << "  gaze: " << robot.gazeJoint() << std::endl;
  os << "  waist: " << robot.waist() << std::endl;
  os << "  chest: " << robot.waist() << std::endl;
  os << "  left wrist: " << robot.leftWrist() << std::endl;
  os << "  right wrist: " << robot.rightWrist() << std::endl;
  os << "  left ankle: " << robot.leftAnkle() << std::endl;
  os << "  right ankle: " << robot.rightAnkle() << std::endl;
  os << std::endl;
  os << "  gaze origin: " << robot.gazeOrigin() << std::endl;
  os << "  gaze direction: " << robot.gazeDirection() << std::endl;
  os << std::endl;
  os << "  left hand" << std::endl;
  CjrlHand* hand = robot.leftHand();
  vector3d v;
  hand-> getCenter(v);
  os << "    center: " << v << std::endl;
  hand-> getThumbAxis(v);
  os << "    thumb axis: " << v << std::endl;
  hand-> getForeFingerAxis(v);
  os << "    forefinger axis: " << v << std::endl;
  hand-> getPalmNormal(v);
  os << "    palm normal: " << v << std::endl;
  os << "  right hand" << std::endl;
  hand = robot.rightHand();
  hand-> getCenter(v);
  os << "    center: " << v << std::endl;
  hand-> getThumbAxis(v);
  os << "    thumb axis: " << v << std::endl;
  hand-> getForeFingerAxis(v);
  os << "    forefinger axis: " << v << std::endl;
  hand-> getPalmNormal(v);
  os << "    palm normal: " << v << std::endl;
  os << "  left foot" << std::endl;
  CjrlFoot* foot = robot.leftFoot();
  double length, width;
  foot->getSoleSize(length, width);
  foot->getAnklePositionInLocalFrame(v);
  os << "    sole length: " << length << std::endl;
  os << "    sole width: " << width << std::endl;
  os << "    ankle position in foot frame: " << v << std::endl;
  os << "  right foot" << std::endl;
  foot = robot.rightFoot();
  foot->getSoleSize(length, width);
  foot->getAnklePositionInLocalFrame(v);
  os << "    sole length: " << length << std::endl;
  os << "    sole width: " << width << std::endl;
  os << "    ankle position in foot frame: " << v << std::endl;
  os << std::endl;
  os << "  Current configuration: " << robot.currentConfiguration()
     << std::endl;
  os << std::endl;
  os << std::endl;
  os << "  Writing kinematic chain" << std::endl;

  //
  // Go through joints and output each joint
  //
  CjrlJoint* joint = robot.rootJoint();

  if (joint) {
    os << *joint << std::endl;
  }
  // Get position of center of mass
  MAL_S3_VECTOR(com, double);
  com = robot.positionCenterOfMass();

  //debug
  os <<"total mass "<<robot.mass() <<", COM: "
      << MAL_S3_VECTOR_ACCESS(com, 0)
      <<", "<< MAL_S3_VECTOR_ACCESS(com, 1)
      <<", "<< MAL_S3_VECTOR_ACCESS(com, 2)
      <<std::endl;
  */
  return os;
}

std::ostream& sot::operator<<(std::ostream& os, const CjrlJoint& joint)
{
  os << "CjrlJoint:" << &joint << std::endl;
  os << "Rank in configuration "
     << joint.rankInConfiguration() << std::endl;
  os << "Current transformation:" << std::endl;
  os << joint.currentTransformation() << std:: endl;
  os << std::endl;

  CjrlBody* body = joint.linkedBody();
  if (body) {
    os << "Attached body:" << std::endl;
    os << "Mass of the attached body: " << body->mass() << std::endl;
    os << "Local center of mass:" << body->localCenterOfMass() << std::endl;
    os << "Inertia matrix:" << std::endl;
    os << body->inertiaMatrix() << std::endl;
  } else {
    os << "No attached body" << std::endl;
  }

  for (unsigned int iChild=0; iChild < joint.countChildJoints();
       iChild++) {
    os << *(joint.childJoint(iChild)) << std::endl;
    os <<std::endl;
  }

  return os;
}
