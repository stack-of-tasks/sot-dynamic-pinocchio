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
  ,m_model(NULL)
  ,m_data(NULL)

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
    "    Display the current robot configuration.\n"
    "\n"
    "      Input:\n"
    "        - none \n"
    "\n";
  addCommand("displayModel",
	     new command::DisplayModel(*this, docstring));
  docstring = "    \n"
    "    Get the dimension of the robot configuration.\n"
    "    \n"
    "      Return:\n"
    "        an unsigned int: the dimension.\n"
    "    \n";
  addCommand("getDimension",
	     new command::GetDimension(*this, docstring));

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
  
  sotDEBUG(10)<< "Dynamic class_name address"<<&CLASS_NAME<<std::endl;
  sotDEBUGOUT(5);
}

Dynamic::~Dynamic( void ) {
  sotDEBUGIN(15);
    //  if (0!=m_model){ delete m_model; m_model=NULL;}
  if (0!=m_data)
    { delete m_data; m_data=NULL; }
  if (0!=m_model)
    { delete m_model; m_model=NULL; }

  for( std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
       iter != genericSignalRefs.end();
       ++iter) {
    SignalBase<int>* sigPtr = *iter;
    delete sigPtr;
  }
  sotDEBUGOUT(15);
}

/*--------------------------------GETTERS-------------------------------------------*/

dg::Vector& Dynamic::
getLowerPositionLimits(dg::Vector& res, const int&)
{
 
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nq);
  res = m_model->lowerPositionLimit;

  sotDEBUG(15) << "lowerLimit (" << res << ")=" << std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getUpperPositionLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nq);
  res = m_model->upperPositionLimit;

  sotDEBUG(15) << "upperLimit (" << res << ")=" <<std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getUpperVelocityLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  res = m_model->velocityLimit;

  sotDEBUG(15) << "upperVelocityLimit (" << res << ")=" <<std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& Dynamic::
getMaxEffortLimits(dg::Vector& res, const int&)
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  res = m_model->effortLimit;

  sotDEBUGOUT(15);
  return res;
}


/* ---------------- INTERNAL ------------------------------------------------ */
dg::Vector Dynamic::getPinocchioPos(int time)
{
  sotDEBUGIN(15);
  dg::Vector qJoints=jointPositionSIN.access(time);
  dg::Vector q;
  assert(m_model);

  if( freeFlyerPositionSIN) {
    dg::Vector qFF=freeFlyerPositionSIN.access(time);


    q.resize(qJoints.size() + 7);

    Eigen::Quaternion<double> quat = Eigen::AngleAxisd(qFF(5),Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(qFF(4),Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(qFF(3),Eigen::Vector3d::UnitX());
    
    q << qFF(0),qFF(1),qFF(2),
      quat.x(),quat.y(),quat.z(),quat.w(),
      qJoints;
  }
  else if (se3::nv(m_model->joints[1]) == 6){
    dg::Vector qFF = qJoints.head<6>();
    q.resize(qJoints.size()+1);

    Eigen::Quaternion<double> quat = Eigen::AngleAxisd(qFF(5),Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(qFF(4),Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(qFF(3),Eigen::Vector3d::UnitX());
    q << qFF(0),qFF(1),qFF(2),
      quat.x(),quat.y(),quat.z(),quat.w(),
      qJoints.segment(6,qJoints.size()-6);

    assert(q.size() == m_model->nq);
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
  else 
    return vJoints;
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
  assert(m_model);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model->existFrame(jointName)) {
    int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericJacobian,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    int jointId = m_model->getJointId(jointName);
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
  assert(m_model);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model->existFrame(jointName)) {
    int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&Dynamic::computeGenericEndeffJacobian,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrix)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    int jointId = m_model->getJointId(jointName); 
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
  assert(m_model);
  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig;
  if(m_model->existFrame(jointName)) {
    int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< MatrixHomogeneous,int >
      ( boost::bind(&Dynamic::computeGenericPosition,this,true,frameId,_1,_2),
	newtonEulerSINTERN,
	"sotDynamic("+name+")::output(matrixHomo)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    int jointId = m_model->getJointId(jointName); 
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
  assert(m_model);
  int jointId = m_model->getJointId(jointName);

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
  assert(m_model);
  int jointId = m_model->getJointId(jointName);
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
    assert(m_data);
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
  assert(m_model);
  assert(m_data);
  newtonEulerSINTERN(time);
  res.resize(6,m_model->nv);
  se3::computeJacobians(*m_model,*m_data,this->getPinocchioPos(time));

  se3::Data::Matrix6x m_output = Eigen::MatrixXd::Zero(6,m_model->nv);
  //Computes Jacobian in world coordinates. 
  if(isFrame){
    se3::framesForwardKinematics(*m_model,*m_data);
    se3::getFrameJacobian<false>(*m_model,*m_data,(se3::Model::Index)jointId,m_output);
  }
  else se3::getJacobian<false>(*m_model,*m_data,(se3::Model::Index)jointId,m_output);
  res = m_output;
  sotDEBUGOUT(25);
  return res;
}

dg::Matrix& Dynamic::
computeGenericEndeffJacobian(bool isFrame, int jointId,dg::Matrix& res,int time )
{
  sotDEBUGIN(25);
  assert(m_model);
  assert(m_data);
  newtonEulerSINTERN(time);
  res.resize(6,m_model->nv);
  //In local coordinates.
  se3::computeJacobians(*m_model,*m_data,this->getPinocchioPos(time));
  se3::Data::Matrix6x m_output = Eigen::MatrixXd::Zero(6,m_model->nv);

  if(isFrame){
    se3::framesForwardKinematics(*m_model,*m_data);
    se3::getFrameJacobian<true>(*m_model,*m_data,(se3::Model::Index)jointId,m_output);
  }
  else se3::getJacobian<true>(*m_model,*m_data,(se3::Model::Index)jointId,m_output);

  res = m_output;
  sotDEBUGOUT(25);

  return res;
}

MatrixHomogeneous& Dynamic::
computeGenericPosition(bool isFrame, int jointId, MatrixHomogeneous& res, int time)
{
  sotDEBUGIN(25);
  assert(m_data);
  newtonEulerSINTERN(time);
  if(isFrame){
    //TODO: Confirm if we need this. Already being called when calculating jacobian
    //se3::framesForwardKinematics(m_model,*m_data);
    res.matrix()= m_data->oMi[jointId].toHomogeneousMatrix();
  }
  else res.matrix()= m_data->oMi[jointId].toHomogeneousMatrix();

  sotDEBUGOUT(25);
  return res;
}

dg::Vector& Dynamic::
computeGenericVelocity( int jointId, dg::Vector& res,int time )
{
  sotDEBUGIN(25);
  assert(m_data);
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
  assert(m_data);
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
  assert(m_model);
  assert(m_data);
  
  const Eigen::VectorXd q=getPinocchioPos(time);
  const Eigen::VectorXd v=getPinocchioVel(time);
  const Eigen::VectorXd a=getPinocchioAcc(time);
  se3::rnea(*m_model,*m_data,q,v,a);
  
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

  Jcom = se3::jacobianCenterOfMass(*m_model, *m_data,
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
  com = se3::centerOfMass(*m_model,*m_data,q,false,true);
  sotDEBUGOUT(25);
  return com;
}

dg::Matrix& Dynamic::
computeInertia( dg::Matrix& res,int time )
{
    sotDEBUGIN(25);
    newtonEulerSINTERN(time);
    //TODO: USE CCRBA
    dg::Matrix upperInertiaMatrix = se3::crba(*m_model,
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
  if(!m_model->existJointName("r_sole_joint")) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
			       "Robot has no joint corresponding to rigthFoot");
  }
  int jointId = m_model->getJointId("r_sole_joint");
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
  if( cmdLine == "displayModel" ) {
    displayModel();
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
       << "  - displayModel\t:display the current model configuration" <<std::endl
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
