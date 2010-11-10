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

#include <jrl/mal/matrixabstractlayer.hh>

#include <sot-dynamic/dynamic.h>
#include <sot-core/debug.h>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Dynamic,"Dynamic");

using namespace std;

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
{
  sotDEBUGIN(5);

  if( build ) buildModel();

  firstSINTERN.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
  //DEBUG: Why =0? should be function. firstSINTERN.setConstant(0);

  signalRegistration( jointPositionSIN<<freeFlyerPositionSIN
		      <<jointVelocitySIN<<freeFlyerVelocitySIN
		      <<jointAccelerationSIN<<freeFlyerAccelerationSIN);
  signalRegistration( zmpSOUT<<comSOUT<<JcomSOUT<<footHeightSOUT);
	signalRegistration(upperJlSOUT<<lowerJlSOUT<<inertiaSOUT
			 <<inertiaRealSOUT << inertiaRotorSOUT << gearRatioSOUT );
  signalRegistration( MomentaSOUT << AngularMomentumSOUT );

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
void Dynamic::
parseConfigFiles( void )
{
  sotDEBUGIN(15);
  try
    {
      sotDEBUG(35) << "Parse the vrml."<<endl;
      string RobotFileName = vrmlDirectory+vrmlMainFile;
      djj::parseOpenHRPVRMLFile(*m_HDR,RobotFileName,xmlRankFile,xmlSpecificityFile);
    }
  catch( ... )
    { SOT_THROW ExceptionDynamic( ExceptionDynamic::DYNAMIC_JRL,
				     "Error while parsing." ); }
  //inertia.init( aHDMB );
  init = true;
  sotDEBUGOUT(15);
}

/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
/* --- SIGNAL ACTIVATION ---------------------------------------------------- */
dg::SignalTimeDependent< ml::Matrix,int > & Dynamic::
createJacobianSignal( const std::string& signame,const unsigned int& bodyRank )
{

  vector<CjrlJoint *> aVec = m_HDR->jointVector();
  if( bodyRank>=aVec.size() )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_RANK,
				     "Joint rank is too high.",
				     "(rank=%d, while creating J signal).",
				     bodyRank );
    }
  CjrlJoint * aJoint = aVec[bodyRank];


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
createEndeffJacobianSignal( const std::string& signame,const unsigned int& bodyRank )
{
  sotDEBUGIN(15);

  vector<CjrlJoint *> aVec = m_HDR->jointVector();
  if( bodyRank>=aVec.size() )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_RANK,
				     "Joint rank is too high.",
				     "(rank=%d, while creating J signal).",
				     bodyRank );
    }
  CjrlJoint * aJoint = aVec[bodyRank];


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
createPositionSignal( const std::string& signame,const unsigned int& bodyRank )
{
  sotDEBUGIN(15);

  vector<CjrlJoint *> aVec = m_HDR->jointVector();

  if( bodyRank>=aVec.size() )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_RANK,
				     "Joint rank is too high.",
				     "(rank=%d, while creating position signal).",
				     bodyRank );
    }
  CjrlJoint * aJoint = aVec[bodyRank];

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
createVelocitySignal( const std::string& signame,const unsigned int& bodyRank )
{
  sotDEBUGIN(15);

  vector<CjrlJoint *> aVec = m_HDR->jointVector();

  if( bodyRank>=aVec.size() )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_RANK,
				     "Joint rank is too high.",
				     "(rank=%d, while creating velocity signal).",
				     bodyRank );
    }
  CjrlJoint * aJoint = aVec[bodyRank];

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
createAccelerationSignal( const std::string& signame,const unsigned int& bodyRank )
{
  sotDEBUGIN(15);

  vector<CjrlJoint *> aVec = m_HDR->jointVector();

  if( bodyRank>=aVec.size() )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_RANK,
				     "Joint rank is too high.",
				     "(rank=%d, while creating acceleration signal).",
				     bodyRank );
    }
  CjrlJoint * aJoint = aVec[bodyRank];

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
				     "Cannot destroy signal",
				     " (while trying to remove generic acc signal <%s>).",
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
computeFootHeight( double& foot,int time )
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
				     "Position vector size uncorrect",
				     " (Vector size is %d, should be %d).",
				     pos.size(),m_HDR->currentConfiguration().size() );
    }


  if(! m_HDR->currentVelocity(vel.accessToMotherLib()) )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_SIZE,
				     "Velocity vector size uncorrect",
				     " (Vector size is %d, should be %d).",
				     vel.size(),m_HDR->currentVelocity().size() );
    }

  if(! m_HDR->currentAcceleration(acc.accessToMotherLib()) )
    {
      SOT_THROW ExceptionDynamic( ExceptionDynamic::JOINT_SIZE,
				     "Acceleration vector size uncorrect",
				     " (Vector size is %d, should be %d).",
				     acc.size(),m_HDR->currentAcceleration().size() );
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
getUpperJointLimits( ml::Vector& res,const int& time )
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
getLowerJointLimits( ml::Vector& res,const int& time )
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
      createJacobianSignal(Jname,rank);
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
      createPositionSignal(Jname,rank);
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
      createVelocitySignal(Jname,rank);
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
      createAccelerationSignal(Jname,rank);
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
      createEndeffJacobianSignal(Jname,rank);
    }
  else if( cmdLine == "createOpPoint" )
    {
      std::string Jname; cmdArgs >> Jname;
      unsigned int rank; cmdArgs >> rank;
      createEndeffJacobianSignal(string("J")+Jname,rank);
      createPositionSignal(Jname,rank);
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



