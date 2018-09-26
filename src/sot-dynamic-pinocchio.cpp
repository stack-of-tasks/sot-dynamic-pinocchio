/*
 * Copyright 2016,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic-pinocchio.
 * sot-dynamic-pinocchio is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic-pinocchio is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic-pinocchio.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <sot/core/debug.hh>

#include <sot-dynamic-pinocchio/dynamic-pinocchio.h>

#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/multibody/model.hpp>

#include <dynamic-graph/all-commands.h>

#include "../src/dynamic-command.h"


using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dg::sot::DynamicPinocchio::CLASS_NAME = "DynamicPinocchio";

DynamicPinocchio::
DynamicPinocchio( const std::string & name)
  :Entity(name)
  ,m_model(NULL)
  ,m_data(NULL)

  ,jointPositionSIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::position")
  ,freeFlyerPositionSIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::ffposition")
  ,jointVelocitySIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::velocity")
  ,freeFlyerVelocitySIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::ffvelocity")
  ,jointAccelerationSIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::acceleration")
  ,freeFlyerAccelerationSIN(NULL,"sotDynamicPinocchio("+name+")::input(vector)::ffacceleration")

  ,pinocchioPosSINTERN( boost::bind(&DynamicPinocchio::getPinocchioPos,this,_1, _2),
                        jointPositionSIN<<freeFlyerPositionSIN,
                        "sotDynamicPinocchio("+name+")::intern(dummy)::pinocchioPos" )
  ,pinocchioVelSINTERN( boost::bind(&DynamicPinocchio::getPinocchioVel,this,_1, _2),
                         jointVelocitySIN<<freeFlyerVelocitySIN,
                         "sotDynamicPinocchio("+name+")::intern(dummy)::pinocchioVel" )
  ,pinocchioAccSINTERN( boost::bind(&DynamicPinocchio::getPinocchioAcc,this,_1, _2),
                        jointAccelerationSIN<<freeFlyerAccelerationSIN,
                        "sotDynamicPinocchio("+name+")::intern(dummy)::pinocchioAcc" )

  ,newtonEulerSINTERN( boost::bind(&DynamicPinocchio::computeNewtonEuler,this,_1,_2),
                       pinocchioPosSINTERN<<pinocchioVelSINTERN<<pinocchioAccSINTERN,
		       "sotDynamicPinocchio("+name+")::intern(dummy)::newtoneuler" )
  ,jacobiansSINTERN( boost::bind(&DynamicPinocchio::computeJacobians,this,_1, _2),
                     pinocchioPosSINTERN,
                     "sotDynamicPinocchio("+name+")::intern(dummy)::computeJacobians" )
  ,forwardKinematicsSINTERN( boost::bind(&DynamicPinocchio::computeForwardKinematics,this,_1, _2),
                             pinocchioPosSINTERN<<pinocchioVelSINTERN<<pinocchioAccSINTERN,
                             "sotDynamicPinocchio("+name+")::intern(dummy)::computeForwardKinematics" )
  ,ccrbaSINTERN( boost::bind(&DynamicPinocchio::computeCcrba,this,_1,_2),
                 pinocchioPosSINTERN<<pinocchioVelSINTERN,
                 "sotDynamicPinocchio("+name+")::intern(dummy)::computeCcrba" )
  ,zmpSOUT( boost::bind(&DynamicPinocchio::computeZmp,this,_1,_2),
	    newtonEulerSINTERN,
	    "sotDynamicPinocchio("+name+")::output(vector)::zmp" )
  ,JcomSOUT( boost::bind(&DynamicPinocchio::computeJcom,this,_1,_2),
             pinocchioPosSINTERN,
	     "sotDynamicPinocchio("+name+")::output(matrix)::Jcom" )
  ,comSOUT( boost::bind(&DynamicPinocchio::computeCom,this,_1,_2),
	    forwardKinematicsSINTERN,
	    "sotDynamicPinocchio("+name+")::output(vector)::com" )
  ,inertiaSOUT( boost::bind(&DynamicPinocchio::computeInertia,this,_1,_2),
                pinocchioPosSINTERN,
		"sotDynamicPinocchio("+name+")::output(matrix)::inertia" )
  ,footHeightSOUT( boost::bind(&DynamicPinocchio::computeFootHeight,this,_1,_2),
                   pinocchioPosSINTERN,
		   "sotDynamicPinocchio("+name+")::output(double)::footHeight" )
  ,upperJlSOUT( boost::bind(&DynamicPinocchio::getUpperPositionLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamicPinocchio("+name+")::output(vector)::upperJl" )

  ,lowerJlSOUT( boost::bind(&DynamicPinocchio::getLowerPositionLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamicPinocchio("+name+")::output(vector)::lowerJl" )

  ,upperVlSOUT( boost::bind(&DynamicPinocchio::getUpperVelocityLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamicPinocchio("+name+")::output(vector)::upperVl" )

  ,upperTlSOUT( boost::bind(&DynamicPinocchio::getMaxEffortLimits,this,_1,_2),
		sotNOSIGNAL,
		"sotDynamicPinocchio("+name+")::output(vector)::upperTl" )

  ,inertiaRotorSOUT( "sotDynamicPinocchio("+name+")::output(matrix)::inertiaRotor" )
  ,gearRatioSOUT( "sotDynamicPinocchio("+name+")::output(matrix)::gearRatio" )
  ,inertiaRealSOUT( boost::bind(&DynamicPinocchio::computeInertiaReal,this,_1,_2),
		    inertiaSOUT << gearRatioSOUT << inertiaRotorSOUT,
		    "sotDynamicPinocchio("+name+")::output(matrix)::inertiaReal" )
  ,MomentaSOUT( boost::bind(&DynamicPinocchio::computeMomenta,this,_1,_2),
		ccrbaSINTERN,
		"sotDynamicPinocchio("+name+")::output(vector)::momenta" )
  ,AngularMomentumSOUT( boost::bind(&DynamicPinocchio::computeAngularMomentum,this,_1,_2),
			ccrbaSINTERN,
			"sotDynamicPinocchio("+name+")::output(vector)::angularmomentum" )
  ,dynamicDriftSOUT( boost::bind(&DynamicPinocchio::computeTorqueDrift,this,_1,_2),
                     newtonEulerSINTERN,
                     "sotDynamicPinocchio("+name+")::output(vector)::dynamicDrift" )
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
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createOpPointSignals,
				docstring));

    docstring = docCommandVoid2("Create a jacobian (world frame) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createJacobian",
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createJacobianWorldSignal,
				docstring));

    docstring = docCommandVoid2("Create a jacobian (endeff frame) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createJacobianEndEff",
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createJacobianEndEffectorSignal,
				docstring));

    docstring = docCommandVoid2("Create a position (matrix homo) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createPosition",
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createPositionSignal,docstring));

    docstring = docCommandVoid2("Create a velocity (vector) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createVelocity",
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createVelocitySignal,docstring));

    docstring = docCommandVoid2("Create an acceleration (vector) signal only for one joint.",
				"string (signal name)","string (joint name)");
    addCommand("createAcceleration",
	       makeCommandVoid2(*this,&DynamicPinocchio::cmd_createAccelerationSignal,docstring));

  }


  sphericalJoints.clear();

  sotDEBUG(10)<< "Dynamic class_name address"<<&CLASS_NAME<<std::endl;
  sotDEBUGOUT(5);
}

DynamicPinocchio::~DynamicPinocchio( void ) {
  sotDEBUGIN(15);
  // TODO currently, m_model and m_data are pointers owned by the Python interpreter
  // so we should not delete them.
  // I (Joseph Mirabel) think it would be wiser to make them belong to this class but
  // I do not know the impact it has.
  //if (0!=m_data ) { delete m_data ; m_data =NULL; }
  //if (0!=m_model) { delete m_model; m_model=NULL; }

  for( std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
       iter != genericSignalRefs.end();
       ++iter) {
    SignalBase<int>* sigPtr = *iter;
    delete sigPtr;
  }
  sotDEBUGOUT(15);
}

void
DynamicPinocchio::setModel(se3::Model* modelPtr){
  this->m_model = modelPtr;

  if (this->m_model->nq > m_model->nv) {
    if (se3::nv(this->m_model->joints[1]) == 6)
      sphericalJoints.push_back(3);  //FreeFlyer Orientation

    for(int i = 1; i<this->m_model->njoints; i++)  //0: universe
      if(se3::nq(this->m_model->joints[i]) == 4) //Spherical Joint Only
	sphericalJoints.push_back(se3::idx_v(this->m_model->joints[i]));
  }
}


void
DynamicPinocchio::setData(se3::Data* dataPtr){
  this->m_data = dataPtr;

}

/*--------------------------------GETTERS-------------------------------------------*/

dg::Vector& DynamicPinocchio::
getLowerPositionLimits(dg::Vector& res, const int&) const
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  if (!sphericalJoints.empty()) {
    int fillingIndex = 0; //SoTValue
    int origIndex = 0;  //PinocchioValue
    for (std::vector<int>::const_iterator it = sphericalJoints.begin();
	 it < sphericalJoints.end(); it++){
      if(*it-fillingIndex > 0){
	res.segment(fillingIndex, *it-fillingIndex) = m_model->lowerPositionLimit.segment(origIndex, *it-fillingIndex);

	//Don't Change this order
	origIndex += *it-fillingIndex;
	fillingIndex += *it-fillingIndex;
      }
      //Found a Spherical Joint.
      //Assuming that spherical joint limits are unset
      res(fillingIndex) = std::numeric_limits<double>::min();
      res(fillingIndex+1) = std::numeric_limits<double>::min();
      res(fillingIndex+2) = std::numeric_limits<double>::min();
      fillingIndex +=3;
      origIndex +=4;
    }
    assert(m_model->nv-fillingIndex == m_model->nq- origIndex);
    if(m_model->nv > fillingIndex)
      res.segment(fillingIndex, m_model->nv-fillingIndex) =
	m_model->lowerPositionLimit.segment(origIndex, m_model->nv-fillingIndex);
  }
  else {
    res = m_model->lowerPositionLimit;
  }
  sotDEBUG(15) << "lowerLimit (" << res << ")=" << std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& DynamicPinocchio::
getUpperPositionLimits(dg::Vector& res, const int&) const
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  if (!sphericalJoints.empty()) {
    int fillingIndex = 0; //SoTValue
    int origIndex = 0;  //PinocchioValue
    for (std::vector<int>::const_iterator it = sphericalJoints.begin();
	 it < sphericalJoints.end(); it++){
      if(*it-fillingIndex > 0){
	res.segment(fillingIndex, *it-fillingIndex) = m_model->upperPositionLimit.segment(origIndex, *it-fillingIndex);

	//Don't Change this order
	origIndex += *it-fillingIndex;
	fillingIndex += *it-fillingIndex;
      }
      //Found a Spherical Joint.
      //Assuming that spherical joint limits are unset
      res(fillingIndex) = std::numeric_limits<double>::max();
      res(fillingIndex+1) = std::numeric_limits<double>::max();
      res(fillingIndex+2) = std::numeric_limits<double>::max();
      fillingIndex +=3;
      origIndex +=4;
    }
    assert(m_model->nv-fillingIndex == m_model->nq- origIndex);
    if(m_model->nv > fillingIndex)
      res.segment(fillingIndex, m_model->nv-fillingIndex) =
	m_model->upperPositionLimit.segment(origIndex, m_model->nv-fillingIndex);
  }
  else {
    res = m_model->upperPositionLimit;
  }
  sotDEBUG(15) << "upperLimit (" << res << ")=" << std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& DynamicPinocchio::
getUpperVelocityLimits(dg::Vector& res, const int&) const
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  res = m_model->velocityLimit;

  sotDEBUG(15) << "upperVelocityLimit (" << res << ")=" <<std::endl;
  sotDEBUGOUT(15);
  return res;
}

dg::Vector& DynamicPinocchio::
getMaxEffortLimits(dg::Vector& res, const int&) const
{
  sotDEBUGIN(15);
  assert(m_model);

  res.resize(m_model->nv);
  res = m_model->effortLimit;

  sotDEBUGOUT(15);
  return res;
}


/* ---------------- INTERNAL ------------------------------------------------ */
dg::Vector& DynamicPinocchio::getPinocchioPos(dg::Vector& q,const int& time)
{
  sotDEBUGIN(15);
  dg::Vector qJoints=jointPositionSIN.access(time);
  if (!sphericalJoints.empty()){
    if( freeFlyerPositionSIN) {
      dg::Vector qFF=freeFlyerPositionSIN.access(time);
      qJoints.head<6>() = qFF;  //Overwrite qJoints ff with ffposition value
      assert(sphericalJoints[0] == 3); // FreeFlyer should ideally be present.
    }
    q.resize(qJoints.size()+sphericalJoints.size());
    int fillingIndex = 0;
    int origIndex = 0;
    for (std::vector<int>::const_iterator it = sphericalJoints.begin(); it < sphericalJoints.end(); it++){
      if(*it-origIndex > 0){
	q.segment(fillingIndex,*it-origIndex) = qJoints.segment(origIndex,*it-origIndex);
	fillingIndex += *it-origIndex;
	origIndex += *it-origIndex;
      }
      assert(*it == origIndex);
      Eigen::Quaternion<double> temp =
	Eigen::AngleAxisd(qJoints(origIndex+2),Eigen::Vector3d::UnitZ())*
	Eigen::AngleAxisd(qJoints(origIndex+1),Eigen::Vector3d::UnitY())*
	Eigen::AngleAxisd(qJoints(origIndex),Eigen::Vector3d::UnitX());
      q(fillingIndex) = temp.x();
      q(fillingIndex+1) = temp.y();
      q(fillingIndex+2) = temp.z();
      q(fillingIndex+3) = temp.w();
      fillingIndex +=4;
      origIndex +=3;
    }
    if(qJoints.size()>origIndex) q.segment(fillingIndex, qJoints.size()-origIndex) = qJoints.tail(qJoints.size()-origIndex);
  }
  else {
    q.resize(qJoints.size());
    q=qJoints;
  }

  sotDEBUG(15) <<"Position out"<<q<<std::endl;
  sotDEBUGOUT(15);
  return q;
}

dg::Vector& DynamicPinocchio::getPinocchioVel(dg::Vector& v, const int& time)
{
  const Eigen::VectorXd vJoints=jointVelocitySIN.access(time);
  if(freeFlyerVelocitySIN){
    const Eigen::VectorXd vFF=freeFlyerVelocitySIN.access(time);
    if(v.size() != vJoints.size() + vFF.size())
      v.resize(vJoints.size() + vFF.size());
    v << vFF,vJoints;
    return v;
  }
  else {
    v = vJoints;
    return v;
  }
}

dg::Vector& DynamicPinocchio::getPinocchioAcc(dg::Vector& a, const int& time)
{
  const Eigen::VectorXd aJoints=jointAccelerationSIN.access(time);
  if(freeFlyerAccelerationSIN){
    const Eigen::VectorXd aFF=freeFlyerAccelerationSIN.access(time);
    if (a.size() !=aJoints.size() + aFF.size())
      a.resize(aJoints.size() + aFF.size());
    a << aFF,aJoints;
    return a;
  }
  else {
    a = aJoints;
    return a;
  }
}

/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
dg::SignalTimeDependent< dg::Matrix,int > & DynamicPinocchio::
createJacobianSignal( const std::string& signame, const std::string& jointName )
{
  sotDEBUGIN(15);
  assert(m_model);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model->existFrame(jointName)) {
    long int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&DynamicPinocchio::computeGenericJacobian,this,true,frameId,_1,_2),
	jacobiansSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrix)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    long int jointId = m_model->getJointId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&DynamicPinocchio::computeGenericJacobian,this,false,jointId,_1,_2),
	jacobiansSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrix)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< dg::Matrix,int > & DynamicPinocchio::
createEndeffJacobianSignal( const std::string& signame, const std::string& jointName )
{
  sotDEBUGIN(15);
  assert(m_model);
  dg::SignalTimeDependent< dg::Matrix,int > * sig;
  if(m_model->existFrame(jointName)) {
    long int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&DynamicPinocchio::computeGenericEndeffJacobian,this,true,frameId,_1,_2),
	jacobiansSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrix)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    long int jointId = m_model->getJointId(jointName);
    sig = new dg::SignalTimeDependent< dg::Matrix,int >
      ( boost::bind(&DynamicPinocchio::computeGenericEndeffJacobian,this,false,jointId,_1,_2),
	jacobiansSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrix)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< MatrixHomogeneous,int >& DynamicPinocchio::
createPositionSignal( const std::string& signame, const std::string& jointName)
{
  sotDEBUGIN(15);
  assert(m_model);
  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig;
  if(m_model->existFrame(jointName)) {
    long int frameId = m_model->getFrameId(jointName);
    sig = new dg::SignalTimeDependent< MatrixHomogeneous,int >
      ( boost::bind(&DynamicPinocchio::computeGenericPosition,this,true,frameId,_1,_2),
	forwardKinematicsSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrixHomo)::"+signame );
  }
  else if(m_model->existJointName(jointName)) {
    long int jointId = m_model->getJointId(jointName);
    sig = new dg::SignalTimeDependent< MatrixHomogeneous,int >
      ( boost::bind(&DynamicPinocchio::computeGenericPosition,this,false,jointId,_1,_2),
	forwardKinematicsSINTERN,
	"sotDynamicPinocchio("+name+")::output(matrixHomo)::"+signame );
  }
  else SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
				  "Robot has no joint corresponding to " + jointName);

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
  sotDEBUGOUT(15);
  return *sig;
}

SignalTimeDependent< dg::Vector,int >& DynamicPinocchio::
createVelocitySignal( const std::string& signame,const std::string& jointName )
{
  sotDEBUGIN(15);
  assert(m_model);
  long int jointId = m_model->getJointId(jointName);

  SignalTimeDependent< dg::Vector,int > * sig
    = new SignalTimeDependent< dg::Vector,int >
    ( boost::bind(&DynamicPinocchio::computeGenericVelocity,this,jointId,_1,_2),
      forwardKinematicsSINTERN,
      "sotDynamicPinocchio("+name+")::output(dg::Vector)::"+signame );
  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}

dg::SignalTimeDependent< dg::Vector,int >& DynamicPinocchio::
createAccelerationSignal( const std::string& signame, const std::string& jointName)
{
  sotDEBUGIN(15);
  assert(m_model);
  long int jointId = m_model->getJointId(jointName);
  dg::SignalTimeDependent< dg::Vector,int > * sig
    = new dg::SignalTimeDependent< dg::Vector,int >
    ( boost::bind(&DynamicPinocchio::computeGenericAcceleration,this,jointId,_1,_2),
      forwardKinematicsSINTERN,
      "sotDynamicPinocchio("+name+")::output(dg::Vector)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}

void DynamicPinocchio::
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

void DynamicPinocchio::
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

void DynamicPinocchio::
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

void DynamicPinocchio::
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

dg::Vector& DynamicPinocchio::computeZmp( dg::Vector& res,const int& time )
{
    //TODO: To be verified
    sotDEBUGIN(25);
    assert(m_data);
    if (res.size()!=3)
        res.resize(3);
    newtonEulerSINTERN(time);

    const se3::Force& ftau = m_data->oMi[1].act(m_data->f[1]);
    const se3::Force::Vector3& tau = ftau.angular();
    const se3::Force::Vector3& f = ftau.linear();
    res(0) = -tau[1]/f[2];
    res(1) = tau[0]/f[2];
    res(2) = 0;

    sotDEBUGOUT(25);

    return res;
}

//In world coordinates


//Updates the jacobian matrix in m_data
int& DynamicPinocchio::computeJacobians(int& dummy, const int& time) {
  sotDEBUGIN(25);
  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
  se3::computeJacobians(*m_model,*m_data, q);
  sotDEBUG(25)<<"Jacobians updated"<<std::endl;
  sotDEBUGOUT(25);
  return dummy;
}
int& DynamicPinocchio::computeForwardKinematics(int& dummy, const int& time)  {
  sotDEBUGIN(25);
  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
  const Eigen::VectorXd& v = pinocchioVelSINTERN.access(time);
  const Eigen::VectorXd& a = pinocchioAccSINTERN.access(time);
  se3::forwardKinematics(*m_model, *m_data, q, v, a);
  sotDEBUG(25)<<"Kinematics updated"<<std::endl;
  sotDEBUGOUT(25);
  return dummy;
}

int& DynamicPinocchio::computeCcrba(int& dummy, const int& time) {
  sotDEBUGIN(25);
  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
  const Eigen::VectorXd& v = pinocchioVelSINTERN.access(time);
  se3::ccrba(*m_model,*m_data, q, v);
  sotDEBUG(25)<<"Inertia and Momentum updated"<<std::endl;
  sotDEBUGOUT(25);
  return dummy;
}

dg::Matrix& DynamicPinocchio::
computeGenericJacobian(const bool isFrame, const int jointId, dg::Matrix& res,const int& time )
{
  sotDEBUGIN(25);
  assert(m_model);
  assert(m_data);
  if(res.rows()!=6 && res.cols()!=m_model->nv)
    res.resize(6,m_model->nv);
  jacobiansSINTERN(time);

  //TODO: Find a way to remove tmp object
  se3::Data::Matrix6x tmp = Eigen::MatrixXd::Zero(6,m_model->nv);

  //Computes Jacobian in world coordinates.
  if(isFrame){
    se3::getJacobian<se3::WORLD>(*m_model,*m_data,
                                 m_model->frames[(se3::Model::Index)jointId].parent,tmp);
  }
  else
    se3::getJacobian<se3::WORLD>(*m_model,*m_data,(se3::Model::Index)jointId,tmp);
  res = tmp;
  sotDEBUGOUT(25);
  return res;
}

dg::Matrix& DynamicPinocchio::
computeGenericEndeffJacobian(const bool isFrame, const int jointId,dg::Matrix& res,const int& time )
{
  sotDEBUGIN(25);
  assert(m_model);
  assert(m_data);
  if(res.rows()!=6 && res.cols()!=m_model->nv)
    res.resize(6,m_model->nv);
  jacobiansSINTERN(time);

  //TODO: Find a way to remove tmp object
  se3::Data::Matrix6x tmp = Eigen::MatrixXd::Zero(6,m_model->nv);
  //std::string temp;
  //Computes Jacobian in end-eff coordinates.
  if(isFrame){
    se3::framesForwardKinematics(*m_model,*m_data);
    se3::getFrameJacobian(*m_model,*m_data,(se3::Model::Index)jointId,tmp);
    sotDEBUG(25) << "EndEffJacobian for "
                 << m_model->frames.at((se3::Model::Index)jointId).name
                 <<" is "<<tmp<<std::endl;
  }
  else {
    //temp = m_model->getJointName((se3::Model::Index)jointId);
    se3::getJacobian<se3::LOCAL>
      (*m_model,*m_data,(se3::Model::Index)jointId,tmp);
    sotDEBUG(25) << "EndEffJacobian for "
                 << m_model->getJointName((se3::Model::Index)jointId)
                 <<" is "<<tmp<<std::endl;
  }
  res = tmp;

  sotDEBUGOUT(25);
  return res;
}

MatrixHomogeneous& DynamicPinocchio::
computeGenericPosition(const bool isFrame, const int jointId, MatrixHomogeneous& res, const int& time)
{
  sotDEBUGIN(25);
  assert(m_data);
  std::string temp;
  forwardKinematicsSINTERN(time);
  if(isFrame){
    se3::framesForwardKinematics(*m_model,*m_data);
    res.matrix()= m_data->oMf[jointId].toHomogeneousMatrix();
    temp = m_model->frames.at((se3::Model::Index)jointId).name;
  }
  else{
    res.matrix()= m_data->oMi[jointId].toHomogeneousMatrix();
    temp = m_model->getJointName((se3::Model::Index)jointId);
  }
  sotDEBUG(25)<<"For "<<temp<<" with id: "<<jointId<<" position is "<<res<<std::endl;
  sotDEBUGOUT(25);
  return res;
}

dg::Vector& DynamicPinocchio::
computeGenericVelocity( const int jointId, dg::Vector& res,const int& time )
{
  sotDEBUGIN(25);
  assert(m_data);
  res.resize(6);
  forwardKinematicsSINTERN(time);
  const se3::Motion& aRV = m_data->v[jointId];
  res<<aRV.linear(),aRV.angular();
  sotDEBUGOUT(25);
  return res;
}

dg::Vector& DynamicPinocchio::
computeGenericAcceleration( const int jointId ,dg::Vector& res,const int& time )
{
  sotDEBUGIN(25);
  assert(m_data);
  res.resize(6);
  forwardKinematicsSINTERN(time);
  const se3::Motion& aRA = m_data->a[jointId];
  res<<aRA.linear(),aRA.angular();
  sotDEBUGOUT(25);
  return res;
}

int& DynamicPinocchio::
computeNewtonEuler(int& dummy,const int& time )
{
  sotDEBUGIN(15);
  assert(m_model);
  assert(m_data);
  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
  const Eigen::VectorXd& v = pinocchioVelSINTERN.access(time);
  const Eigen::VectorXd& a = pinocchioAccSINTERN.access(time);
  se3::rnea(*m_model,*m_data,q,v,a);

  sotDEBUG(1)<< "pos = " <<q <<std::endl;
  sotDEBUG(1)<< "vel = " <<v <<std::endl;
  sotDEBUG(1)<< "acc = " <<a <<std::endl;

  sotDEBUGOUT(15);
  return dummy;
}

dg::Matrix& DynamicPinocchio::
computeJcom( dg::Matrix& Jcom,const int& time )
{

  sotDEBUGIN(25);

  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
  Jcom = se3::jacobianCenterOfMass(*m_model, *m_data,
				   q, false);
  sotDEBUGOUT(25);
  return Jcom;
}

dg::Vector& DynamicPinocchio::
computeCom( dg::Vector& com,const int& time )
{

  sotDEBUGIN(25);
  forwardKinematicsSINTERN(time);
  se3::centerOfMass(*m_model,*m_data,false);
  com = m_data->com[0];
  sotDEBUGOUT(25);
  return com;
}

dg::Matrix& DynamicPinocchio::
computeInertia( dg::Matrix& res,const int& time )
{
    sotDEBUGIN(25);
  const Eigen::VectorXd& q = pinocchioPosSINTERN.access(time);
    res = se3::crba(*m_model, *m_data, q);
    res.triangularView<Eigen::StrictlyLower>() =
      res.transpose().triangularView<Eigen::StrictlyLower>();
    sotDEBUGOUT(25);
    return res;
}

dg::Matrix& DynamicPinocchio::
computeInertiaReal( dg::Matrix& res,const int& time )
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

double& DynamicPinocchio::
computeFootHeight (double &res , const int& )
{
  //Ankle position in local foot frame
  //TODO: Confirm that it is in the foot frame
  sotDEBUGIN(25);
  if(!m_model->existJointName("r_sole_joint")) {
    SOT_THROW ExceptionDynamic(ExceptionDynamic::GENERIC,
			       "Robot has no joint corresponding to rigthFoot");
  }
  long int jointId = m_model->getJointId("r_sole_joint");
  Eigen::Vector3d anklePosInLocalRefFrame= m_data->liMi[jointId].translation();
  // TODO: positive or negative? Current output:negative
  res = anklePosInLocalRefFrame(2);
  sotDEBUGOUT(25);
  return res;
}

dg::Vector& DynamicPinocchio::
computeTorqueDrift( dg::Vector& tauDrift,const int& time )
{
  sotDEBUGIN(25);
  newtonEulerSINTERN(time);
  tauDrift = m_data->tau;
  sotDEBUGOUT(25);
  return tauDrift;
}

dg::Vector& DynamicPinocchio::
computeMomenta(dg::Vector & Momenta, const int& time)
{
  sotDEBUGIN(25);
  ccrbaSINTERN(time);
  if (Momenta.size()!=6)
    Momenta.resize(6);

  Momenta = m_data->hg.toVector_impl();

  sotDEBUGOUT(25) << "Momenta :" << Momenta ;
  return Momenta;
}

dg::Vector& DynamicPinocchio::
computeAngularMomentum(dg::Vector & Momenta, const int& time)
{
  sotDEBUGIN(25);
  ccrbaSINTERN(time);

  if (Momenta.size()!=3)
    Momenta.resize(3);
  return Momenta;
  Momenta = m_data->hg.angular_impl();

  sotDEBUGOUT(25) << "AngularMomenta :" << Momenta ;
  return Momenta;
}

/* ------------------------ SIGNAL CASTING--------------------------------------- */

dg::SignalTimeDependent<dg::Matrix,int>& DynamicPinocchio::
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
dg::SignalTimeDependent<MatrixHomogeneous,int>& DynamicPinocchio::
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

dg::SignalTimeDependent<dg::Vector,int>& DynamicPinocchio::
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

dg::SignalTimeDependent<dg::Vector,int>& DynamicPinocchio::
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

//jointName is either a fixed-joint (pinocchio operational frame) or a
//movable joint (pinocchio joint-variant).
void DynamicPinocchio::cmd_createOpPointSignals( const std::string& opPointName,
					const std::string& jointName )
{
    createEndeffJacobianSignal(std::string("J")+opPointName,jointName);
    createPositionSignal(opPointName,jointName);
}
void DynamicPinocchio::cmd_createJacobianWorldSignal( const std::string& signalName,
					     const std::string& jointName )
{
    createJacobianSignal(signalName, jointName);
}
void DynamicPinocchio::cmd_createJacobianEndEffectorSignal( const std::string& signalName,
					     const std::string& jointName )
{
    createEndeffJacobianSignal(signalName, jointName);
}
void DynamicPinocchio::cmd_createPositionSignal( const std::string& signalName,
					const std::string& jointName )
{
    createPositionSignal(signalName, jointName);
}
void DynamicPinocchio::cmd_createVelocitySignal( const std::string& signalName,
          				const std::string& jointName )
{
    createVelocitySignal(signalName, jointName);
}
void DynamicPinocchio::cmd_createAccelerationSignal( const std::string& signalName,
          				const std::string& jointName )
{
    createAccelerationSignal(signalName, jointName);
}
