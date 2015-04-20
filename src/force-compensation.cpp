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

#include <sot-dynamic/force-compensation.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/core/macros-signal.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ForceCompensationPlugin,"ForceCompensation");

/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */
ForceCompensation::
ForceCompensation(void)
  :usingPrecompensation(false)
{}


ForceCompensationPlugin::
ForceCompensationPlugin( const std::string & name )
  :Entity(name)
  ,calibrationStarted(false)

  ,torsorSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::torsorIN")
  ,worldRhandSIN(NULL,"sotForceCompensation("+name+")::input(MatrixRotation)::worldRhand")

  ,handRsensorSIN(NULL,"sotForceCompensation("+name+")::input(MatrixRotation)::handRsensor")
  ,translationSensorComSIN(NULL,"sotForceCompensation("+name+")::input(vector3)::sensorCom")
  ,gravitySIN(NULL,"sotForceCompensation("+name+")::input(vector6)::gravity")
  ,precompensationSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::precompensation")
  ,gainSensorSIN(NULL,"sotForceCompensation("+name+")::input(matrix6)::gain")
  ,deadZoneLimitSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::deadZoneLimit")
  ,transSensorJointSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::sensorJoint")  ,inertiaJointSIN(NULL,"sotForceCompensation("+name+")::input(matrix6)::inertiaJoint")
  ,velocitySIN(NULL,"sotForceCompensation("+name+")::input(vector6)::velocity")
  ,accelerationSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::acceleration")

  ,handXworldSOUT( SOT_INIT_SIGNAL_2( ForceCompensation::computeHandXworld,
				      worldRhandSIN,MatrixRotation,
				      translationSensorComSIN,ml::Vector ),
		   "sotForceCompensation("+name+")::output(MatrixForce)::handXworld" )
   ,handVsensorSOUT( SOT_INIT_SIGNAL_1( ForceCompensation::computeHandVsensor,
					handRsensorSIN,MatrixRotation),
		     "sotForceCompensation("+name+")::output(MatrixForce)::handVsensor" )
   ,torsorDeadZoneSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::torsorNullifiedIN")

  ,sensorXhandSOUT( SOT_INIT_SIGNAL_2( ForceCompensation::computeSensorXhand,
				       handRsensorSIN,MatrixRotation,
				       transSensorJointSIN,ml::Vector ),
		   "sotForceCompensation("+name+")::output(MatrixForce)::sensorXhand" )
//   ,inertiaSensorSOUT( SOT_INIT_SIGNAL_2( ForceCompensation::computeInertiaSensor,
// 					  inertiaJointSIN,ml::Matrix,
// 					  sensorXhandSOUT,MatrixForce ),
// 		       "ForceCompensation("+name+")::output(MatrixForce)::inertiaSensor" )
   ,momentumSOUT( SOT_INIT_SIGNAL_4(ForceCompensation::computeMomentum,
				    velocitySIN,ml::Vector,
				    accelerationSIN,ml::Vector,
				    sensorXhandSOUT,MatrixForce,
				    inertiaJointSIN,ml::Matrix),
		  "sotForceCompensation("+name+")::output(Vector6)::momentum" )
  ,momentumSIN(NULL,"sotForceCompensation("+name+")::input(vector6)::momentumIN")

  ,torsorCompensatedSOUT( SOT_INIT_SIGNAL_7(ForceCompensation::computeTorsorCompensated,
					    torsorSIN,ml::Vector,
					    precompensationSIN,ml::Vector,
					    gravitySIN,ml::Vector,
					    handXworldSOUT,MatrixForce,
					    handVsensorSOUT,MatrixForce,
					    gainSensorSIN,ml::Matrix,
					    momentumSIN,ml::Vector),
			  "sotForceCompensation("+name+")::output(Vector6)::torsor" )
  ,torsorDeadZoneSOUT( SOT_INIT_SIGNAL_2(ForceCompensation::computeDeadZone,
					 torsorDeadZoneSIN,ml::Vector,
					 deadZoneLimitSIN,ml::Vector),
		       "sotForceCompensation("+name+")::output(Vector6)::torsorNullified" )
   ,calibrationTrigerSOUT( boost::bind(&ForceCompensationPlugin::calibrationTriger,
				       this,_1,_2),
			   torsorSIN << worldRhandSIN,
			   "sotForceCompensation("+name+")::output(Dummy)::calibrationTriger")
{
  sotDEBUGIN(5);

  signalRegistration(torsorSIN);
  signalRegistration(worldRhandSIN);
  signalRegistration(handRsensorSIN);
  signalRegistration(translationSensorComSIN);
  signalRegistration(gravitySIN);
  signalRegistration(precompensationSIN);
  signalRegistration(gainSensorSIN);
  signalRegistration(deadZoneLimitSIN);
  signalRegistration(transSensorJointSIN);
  signalRegistration(inertiaJointSIN);
  signalRegistration(velocitySIN );
  signalRegistration(accelerationSIN);
  signalRegistration(handXworldSOUT);
  signalRegistration(handVsensorSOUT);
  signalRegistration(torsorDeadZoneSIN);
  signalRegistration(sensorXhandSOUT);
  signalRegistration(momentumSOUT);
  signalRegistration(momentumSIN);
  signalRegistration(torsorCompensatedSOUT);
  signalRegistration(torsorDeadZoneSOUT);
  signalRegistration(calibrationTrigerSOUT);
  torsorDeadZoneSIN.plug(&torsorCompensatedSOUT);

  // By default, I choose: momentum is not compensated.
  //  momentumSIN.plug( &momentumSOUT );
  ml::Vector v(6); v.fill(0); momentumSIN = v;

  sotDEBUGOUT(5);
}


ForceCompensationPlugin::
~ForceCompensationPlugin( void )
{
  return;
}

/* --- CALIB --------------------------------------------------------------- */
/* --- CALIB --------------------------------------------------------------- */
/* --- CALIB --------------------------------------------------------------- */

MatrixRotation ForceCompensation::I3;

void ForceCompensation::
clearCalibration( void )
{
  torsorList.clear();
  rotationList.clear();
}


void ForceCompensation::
addCalibrationValue( const ml::Vector& /*torsor*/,
		     const MatrixRotation & /*worldRhand*/ )
{
  sotDEBUGIN(45);

  //   sotDEBUG(25) << "Add torsor: t"<<torsorList.size() <<" = " << torsor;
  //   sotDEBUG(25) << "Add Rotation: wRh"<<torsorList.size() <<" = " << worldRhand;

  //   torsorList.push_back(torsor);
  //   rotationList.push_back(worldRhand);

  sotDEBUGOUT(45);
}

ml::Vector ForceCompensation::
calibrateTransSensorCom( const ml::Vector& gravity,
			 const MatrixRotation& /*handRsensor*/ )
{
  //   sotDEBUGIN(25);

  //   ml::Vector grav3(3);
  //   ml::Vector Rgrav3(3),tau(3),Rtau(3);
  //   for( unsigned int j=0;j<3;++j ) { grav3(j)=gravity(j); }

  //   std::list< ml::Vector >::iterator iterTorsor = torsorList.begin();
  //   std::list< MatrixRotation >::const_iterator iterRotation
  //     = rotationList.begin();


  //   const unsigned int NVAL = torsorList.size();
  //   if( 0==NVAL )
  //     {
  //       ml::Vector zero(3); zero.fill(0);
  //       return zero;
  //     }

  //   if(NVAL!=rotationList.size() )
  //     {
  // 	  // TODO: ERROR THROW
  //     }
  //   ml::Matrix torsors( 3,NVAL );
  //   ml::Matrix gravitys( 3,NVAL );

  //   for( unsigned int i=0;i<NVAL;++i )
  //     {
  //       if( (torsorList.end()==iterTorsor)||(rotationList.end()==iterRotation) )
  // 	{
  // 	  // TODO: ERROR THROW
  // 	  break;
  // 	}
  //       const ml::Vector & torsor = *iterTorsor;
  //       const MatrixRotation & worldRhand = *iterRotation;

  //       for( unsigned int j=0;j<3;++j ) { tau(j)=torsor(j+3); }
  //       handRsensor.multiply(tau,Rtau);
  //       worldRhand.transpose().multiply( grav3,Rgrav3 );
  //       for( unsigned int j=0;j<3;++j )
  // 	{
  // 	  torsors( j,i ) = -Rtau(j);
  // 	  gravitys( j,i ) = Rgrav3(j);
  // 	}
  //       sotDEBUG(35) << "R" << i << " = " << worldRhand;
  //       sotDEBUG(35) << "Rtau" << i << " = " << Rtau;
  //       sotDEBUG(35) << "Rg" << i << " = " << Rgrav3;

  //       iterTorsor++; iterRotation++;
  //     }

  //   sotDEBUG(35) << "Rgs = " << gravitys;
  //   sotDEBUG(35) << "Rtaus = " << torsors;

  //   ml::Matrix gravsInv( gravitys.nbCols(),gravitys.nbRows() );
  //   sotDEBUG(25) << "Compute the pinv..." << std::endl;
  //   gravitys.pseudoInverse(gravsInv);
  //   sotDEBUG(25) << "Compute the pinv... [DONE]" << std::endl;
  //   sotDEBUG(25) << "gravsInv = " << gravsInv << std::endl;

  //   ml::Matrix Skew(3,3);
  //   torsors.multiply( gravsInv,Skew );
  //   sotDEBUG(25) << "Skew = " << Skew << std::endl;

  //   ml::Vector sc(3);
  //   sc(0)=(Skew(2,1)-Skew(1,2))*.5 ;
  //   sc(1)=(Skew(0,2)-Skew(2,0))*.5 ;
  //   sc(2)=(Skew(1,0)-Skew(0,1))*.5 ;
  //   sotDEBUG(15) << "SC = " << sc << std::endl;
  //   /* TAKE the antisym constraint into account inside the minimization pbm. */

  //   sotDEBUGOUT(25);
  //   return sc;
  return gravity;
}

ml::Vector ForceCompensation::
calibrateGravity( const MatrixRotation& /*handRsensor*/,
		  bool /*precompensationCalibration*/,
		  const MatrixRotation& /*hand0RsensorArg*/ )
{
  sotDEBUGIN(25);

  //   MatrixRotation hand0Rsensor;
  //   if( &hand0Rsensor==&I3 ) hand0Rsensor.setIdentity();
  //   else hand0Rsensor=hand0RsensorArg;

  //   std::list< ml::Vector >::const_iterator iterTorsor = torsorList.begin();
  //   std::list< MatrixRotation >::const_iterator iterRotation
  //     = rotationList.begin();

  //   const unsigned int NVAL = torsorList.size();
  //   if(NVAL!=rotationList.size() )
  //     {
  // 	  // TODO: ERROR THROW
  //     }
  //   if( 0==NVAL )
  //     {
  //       ml::Vector zero(6); zero.fill(0);
  //       return zero;
  //     }

  //   ml::Vector force(3),forceHand(3),forceWorld(3);
  //   ml::Vector sumForce(3); sumForce.fill(0);

  //   for( unsigned int i=0;i<NVAL;++i )
  //     {
  //       if( (torsorList.end()==iterTorsor)||(rotationList.end()==iterRotation) )
  // 	{
  // 	  // TODO: ERROR THROW
  // 	  break;
  // 	}
  //       const ml::Vector & torsor = *iterTorsor;
  //       const MatrixRotation & R = *iterRotation;

  //       /* The sensor read [-] the value, and the grav is [-] the sensor force.
  //        * [-]*[-] = [+] -> force = + torsor(1:3). */
  //       for( unsigned int j=0;j<3;++j ) { force(j)=-torsor(j); }
  //       handRsensor.multiply(force,forceHand);
  //       if( usingPrecompensation )
  // 	{
  // 	  ml::Matrix R_I(3,3); R_I = R.transpose();
  // 	  R_I -= hand0Rsensor;
  // 	  R_I.pseudoInverse(.01).multiply( forceHand,forceWorld );
  // 	}
  //       else
  // 	{ R.multiply( forceHand,forceWorld ); }

  //       sotDEBUG(35) << "R(" << i << "*3+1:" << i << "*3+3,:)  = " << R << "';";
  //       sotDEBUG(35) << "rhFs(" << i << "*3+1:" << i << "*3+3) = " << forceHand;
  //       sotDEBUG(45) << "fworld(" << i << "*3+1:" << i << "*3+3) = " << forceWorld;

  //       sumForce+= forceWorld;

  //       iterTorsor++; iterRotation++;
  //     }

  //   sumForce*= (1./NVAL);
  //   sotDEBUG(35) << "Fmean = " << sumForce;
  //   sumForce.resize(6,false);
  //   sumForce(3)=sumForce(4)=sumForce(5)=0.;

  //   sotDEBUG(25)<<"mg = " << sumForce<<std::endl;

  sotDEBUGOUT(25);
  ml::Vector sumForce(3); sumForce.fill(0);
  return sumForce;
}


/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
MatrixForce& ForceCompensation::
computeHandXworld( const MatrixRotation & worldRhand,
		   const ml::Vector & transSensorCom,
		   MatrixForce& res )
{
  sotDEBUGIN(35);

  sotDEBUG(25) << "wRrh = " << worldRhand <<std::endl;
  sotDEBUG(25) << "SC = " << transSensorCom <<std::endl;

  MatrixRotation R; worldRhand.transpose(R);
  MatrixHomogeneous scRw; scRw.buildFrom( R,transSensorCom);
  sotDEBUG(25) << "scMw = " << scRw <<std::endl;

  res.buildFrom( scRw );
  sotDEBUG(15) << "scXw = " << res <<std::endl;

  sotDEBUGOUT(35);
  return res;
}

MatrixForce& ForceCompensation::
computeHandVsensor( const MatrixRotation & handRsensor,
		    MatrixForce& res )
{
  sotDEBUGIN(35);

  for( unsigned int i=0;i<3;++i )
    for( unsigned int j=0;j<3;++j )
      {
	res(i,j)=handRsensor(i,j);
	res(i+3,j+3)=handRsensor(i,j);
	res(i+3,j)=0.;
	res(i,j+3)=0.;
      }

  sotDEBUG(25) << "hVs" << res << std::endl;

  sotDEBUGOUT(35);
  return res;
}

MatrixForce& ForceCompensation::
computeSensorXhand( const MatrixRotation & /*handRsensor*/,
		    const ml::Vector & transJointSensor,
		    MatrixForce& res )
{
  sotDEBUGIN(35);

  /* Force Matrix to pass from the joint frame (ie frame located
   * at the position of the motor, in which the acc is computed by Spong)
   * to the frame SensorHand where all the forces are expressed (ie
   * frame located at the sensor position bu oriented like the hand). */

  MatrixRotation sensorRhand;  sensorRhand.setIdentity();
  //handRsensor.transpose(sensorRhand);
  MatrixHomogeneous sensorMhand;
  sensorMhand.buildFrom( sensorRhand,transJointSensor );

  res.buildFrom( sensorMhand );
  sotDEBUG(25) << "shXJ" << res << std::endl;

  sotDEBUGOUT(35);
  return res;
}

// ml::Matrix& ForceCompensation::
// computeInertiaSensor( const ml::Matrix& inertiaJoint,
// 		      const MatrixForce& sensorXhand,
// 		      ml::Matrix& res )
// {
//   sotDEBUGIN(35);

//   /* Inertia felt at the sensor position, expressed in the orientation
//    * of the hand. */

//   res.resize(6,6);
//   sensorXhand.multiply( inertiaJoint,res );

//   sotDEBUGOUT(35);
//   return res;
// }


ml::Vector& ForceCompensation::
computeTorsorCompensated( const ml::Vector& torqueInput,
			  const ml::Vector& torquePrecompensation,
			  const ml::Vector& gravity,
			  const MatrixForce& handXworld,
			  const MatrixForce& handVsensor,
			  const ml::Matrix& gainSensor,
			  const ml::Vector& momentum,
			  ml::Vector& res )

{
  sotDEBUGIN(35);
  /* Torsor in the sensor frame: K*(-torsred-gamma)+sVh*hXw*mg  */
  /* Torsor in the hand frame: hVs*K*(-torsred-gamma)+hXw*mg  */
  /* With gamma expressed in the sensor frame  (gamma_s = sVh*gamma_h) */

  sotDEBUG(25) << "t_nc = " << torqueInput;
  ml::Vector torquePrecompensated(6);
  //if( usingPrecompensation )
  { torqueInput.addition( torquePrecompensation,torquePrecompensated ); }
  //else { torquePrecompensated = torqueInput; }
  sotDEBUG(25) << "t_pre = " << torquePrecompensated;

  ml::Vector torqueS(6), torqueRH(6);
  gainSensor.multiply( torquePrecompensated,torqueS );
  handVsensor.multiply( torqueS,res );
  sotDEBUG(25) << "t_rh = " << res;

  ml::Vector grh(6);
  handXworld.multiply(gravity,grh);
  grh *= -1;
  sotDEBUG(25) << "g_rh = " << grh;

  res+=grh;
  sotDEBUG(25) << "fcomp = " << res;

  res+=momentum;
  sotDEBUG(25) << "facc = " << res;


  /* TODO res += m xddot */

  sotDEBUGOUT(35);
  return res;
}

ml::Vector& ForceCompensation::
crossProduct_V_F( const ml::Vector& velocity,
		  const ml::Vector& force,
		  ml::Vector& res )
{
  /* [ v;w] x [ f;tau ] = [ w x f; v x f + w x tau ] */
  ml::Vector v(3),w(3),f(3),tau(3);
  for( unsigned int i=0;i<3;++i )
    {
      v(i)=velocity(i); w(i) = velocity(i+3);
      f(i) = force(i); tau(i) = force(i+3);
    }
  ml::Vector res1(3),res2a(3),res2b;
  w.crossProduct( f,res1 );
  v.crossProduct( f,res2a );
  w.crossProduct( tau,res2b );
  res2a+= res2b;

  res.resize(6);
  for( unsigned int i=0;i<3;++i )
    {
      res(i)=res1(i); res(i+3)=res2a(i);
    }
  return res;
}
				

ml::Vector& ForceCompensation::
computeMomentum( const ml::Vector& velocity,
		 const ml::Vector& acceleration,
		 const MatrixForce& sensorXhand,
		 const ml::Matrix& inertiaJoint,
		 ml::Vector& res )
{
  sotDEBUGIN(35);

  /* Fs + Fext = I acc + V x Iv */
  ml::Vector Iacc(6); inertiaJoint.multiply( acceleration,Iacc );
  res.resize(6); sensorXhand.multiply( Iacc,res );

  ml::Vector Iv(6); inertiaJoint.multiply( velocity,Iv);
  ml::Vector vIv(6); crossProduct_V_F( velocity,Iv,vIv );
  ml::Vector XvIv(6); sensorXhand.multiply( vIv,XvIv);
  res+= XvIv;

  sotDEBUGOUT(35);
  return res;
}

ml::Vector& ForceCompensation::
computeDeadZone( const ml::Vector& torqueInput,
		 const ml::Vector& deadZone,
		 ml::Vector& res )
{
  sotDEBUGIN(35);

  if( torqueInput.size()>deadZone.size() ) return res;
  res.resize(torqueInput.size());
  for( unsigned int i=0;i<torqueInput.size();++i )
    {
      const double th = fabs(deadZone(i));
      if( (torqueInput(i)<th)&&(torqueInput(i)>-th) )
	{ res(i)=0; }
      else if(torqueInput(i)<0) res(i)=torqueInput(i)+th;
      else res(i)=torqueInput(i)-th;
    }

  sotDEBUGOUT(35);
  return res;
}


ForceCompensationPlugin::sotDummyType& ForceCompensationPlugin::
calibrationTriger( ForceCompensationPlugin::sotDummyType& dummy,int /*time*/ )
{
  //   sotDEBUGIN(45);
  //   if(! calibrationStarted ) { sotDEBUGOUT(45); return dummy=0; }

  //   addCalibrationValue( torsorSIN(time),worldRhandSIN(time) );
  //   sotDEBUGOUT(45);
  return dummy=1;
}

/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */

void ForceCompensationPlugin::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( "help"==cmdLine )
    {
      os << "ForceCompensation: "
	 << "  - clearCalibration" << std::endl
	 << "  - {start|stop}Calibration [wait <time_sec>]" << std::endl
	 << "  - calibrateGravity\t[only {x|y|z}]" << std::endl
	 << "  - calibratePosition" << std::endl
	 << "  - precomp [{true|false}]:  get/set the "
	 << "precompensation due to sensor calib." << std::endl;
    }
  //   else if( "clearCalibration" == cmdLine )
  //     {
  //       clearCalibration();
  //     }
  //   else if( "startCalibration" == cmdLine )
  //     {
  //       calibrationStarted = true;
  //       cmdArgs >> std::ws;
  //       if( cmdArgs.good() )
  // 	{
  // 	  std::string cmdWait; cmdArgs>>cmdWait>>std::ws;
  // 	  if( (cmdWait == "wait")&&(cmdArgs.good()) )
  // 	    {
  // 	      double timeSec; cmdArgs >> timeSec;
  // 	      unsigned int timeMSec= (unsigned int)(round(timeSec*1000*1000));
  // 	      sotDEBUG(15) << "Calibration: wait for " << timeMSec << "us."<<std::endl;
  // 	      usleep( timeMSec );
  // 	      calibrationStarted = false;
  // 	    }
  // 	}
  //     }
  //   else if( "stopCalibration" == cmdLine )
  //     {
  //       calibrationStarted = false;
  //     }
  //   else if( "calibrateGravity" == cmdLine )
  //     {
  //       if( calibrationStarted )
  // 	{
  // 	  os<< "Calibration phase is on, stop it first."<<std::endl;
  // 	  return;
  // 	}
  //       ml::Vector grav = calibrateGravity( handRsensorSIN.accessCopy(),
  // 					  usingPrecompensation );

  //       cmdArgs >> std::ws;
  //       if( cmdArgs.good() )
  // 	{
  // 	  std::string cmdOnly; cmdArgs>>cmdOnly>>std::ws;
  // 	  if( (cmdOnly == "only")&&(cmdArgs.good()) )
  // 	    {
  // 	      std::string xyz; cmdArgs >> xyz;
  // 	      if( "x"==xyz ) { grav(1)=grav(2)=0.; }
  // 	      else if( "y"==xyz ) { grav(0)=grav(2)=0.; }
  // 	      else if( "z"==xyz ) { grav(0)=grav(1)=0.; }
  // 	    }
  // 	}

  //       gravitySIN = grav;
  //     }
  //   else if( "calibratePosition" == cmdLine )
  //     {
  //       if( calibrationStarted )
  // 	{
  // 	  return;
  //       	  os<< "Calibration phase is on, stop it first."<<std::endl;
  // 	}
  //       ml::Vector position(3);
  //       position = calibrateTransSensorCom( gravitySIN.accessCopy(),
  // 					  handRsensorSIN.accessCopy() );
  //       transSensorComSIN = position;
  //     }
  else if( "precomp" == cmdLine )
    {
      cmdArgs>>std::ws;
      if( cmdArgs.good() )
	{  cmdArgs >>  usingPrecompensation; }
      else { os << "precompensation = " << usingPrecompensation <<std::endl; }
    }
  else if( "compensateMomentum" == cmdLine )
    {
      cmdArgs>>std::ws;
      if( cmdArgs.good() )
	{
	  bool use;  cmdArgs >> use;
	  if( use ) momentumSIN.plug( &momentumSOUT );
	  else
	    {
	      ml::Vector m(6); m.resize(0); momentumSIN = m;
	    }
	}
      else
	{
	  os << "compensateMomentum = " << (momentumSIN.getPtr()!=&momentumSIN)
	     <<std::endl;
	}
    }
  else{ Entity::commandLine( cmdLine,cmdArgs,os ); }


}

