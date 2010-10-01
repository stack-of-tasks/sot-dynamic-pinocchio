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

#include <sot-dynamic/waist-attitude-from-sensor.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistAttitudeFromSensor,"WaistAttitudeFromSensor");

WaistAttitudeFromSensor::
WaistAttitudeFromSensor( const std::string & name ) 
  :Entity(name)
  ,attitudeSensorSIN(NULL,"sotWaistAttitudeFromSensor("+name+")::input(MatrixRotation)::attitudeIN")
  ,positionSensorSIN(NULL,"sotWaistAttitudeFromSensor("+name+")::input(matrixHomo)::position")
  ,attitudeWaistSOUT( boost::bind(&WaistAttitudeFromSensor::computeAttitudeWaist,this,_1,_2),
		      attitudeSensorSIN<<positionSensorSIN,
		      "sotWaistAttitudeFromSensor("+name+")::output(RPY)::attitude" ) 
{
  sotDEBUGIN(5);

  signalRegistration( attitudeSensorSIN<<positionSensorSIN
		      <<attitudeWaistSOUT );

  sotDEBUGOUT(5);
}


WaistAttitudeFromSensor::
~WaistAttitudeFromSensor( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
VectorRollPitchYaw & WaistAttitudeFromSensor::
computeAttitudeWaist( VectorRollPitchYaw & res,
		      const int& time )
{
  sotDEBUGIN(15);
  
  const MatrixHomogeneous & waistMchest = positionSensorSIN( time );
  const MatrixRotation & worldRchest = attitudeSensorSIN( time );

  MatrixRotation waistRchest; waistMchest.extract(waistRchest);
  MatrixRotation chestRwaist; waistRchest.transpose( chestRwaist );

  MatrixRotation worldrchest;
  worldRchest.multiply( chestRwaist,worldrchest);
  res.fromMatrix(worldrchest);
  sotDEBUGOUT(15);
  return res;
}



/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
void WaistAttitudeFromSensor::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

  if( cmdLine == "help" )
    {
      Entity::commandLine(cmdLine,cmdArgs,os);
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}


/* === WaistPoseFromSensorAndContact ===================================== */
/* === WaistPoseFromSensorAndContact ===================================== */
/* === WaistPoseFromSensorAndContact ===================================== */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistPoseFromSensorAndContact,
			  "WaistPoseFromSensorAndContact");

WaistPoseFromSensorAndContact::
WaistPoseFromSensorAndContact( const std::string & name ) 
  :WaistAttitudeFromSensor(name)
   ,fromSensor(false)
   ,positionContactSIN(NULL,"sotWaistPoseFromSensorAndContact("+name+")::input(matrixHomo)::contact")
   ,positionWaistSOUT( boost::bind(&WaistPoseFromSensorAndContact::computePositionWaist,this,_1,_2),
		       attitudeWaistSOUT<<positionContactSIN,
		       "sotWaistPoseFromSensorAndContact("+name+")::output(RPY+T)::positionWaist" ) 
{
  sotDEBUGIN(5);

  signalRegistration( positionContactSIN<<positionWaistSOUT );

  sotDEBUGOUT(5);
}


WaistPoseFromSensorAndContact::
~WaistPoseFromSensorAndContact( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
ml::Vector& WaistPoseFromSensorAndContact::
computePositionWaist( ml::Vector& res,
		      const int& time )
{
  sotDEBUGIN(15);
  
  const MatrixHomogeneous&  waistMcontact = positionContactSIN( time );
  MatrixHomogeneous contactMwaist; waistMcontact.inverse(contactMwaist);

  res.resize(6);
  for( unsigned int i=0;i<3;++i ) 
    { res(i)=contactMwaist(i,3); }

  if(fromSensor)
    {
      const VectorRollPitchYaw & worldrwaist = attitudeWaistSOUT( time );
      for( unsigned int i=0;i<3;++i ) 
	{ res(i+3)=worldrwaist(i); }
    }
  else
    {
      MatrixRotation contactRwaist; 
      contactMwaist.extract(contactRwaist);
      VectorRollPitchYaw contactrwaist; 
      contactrwaist.fromMatrix( contactRwaist );
      
      for( unsigned int i=0;i<3;++i ) 
	{ res(i+3)=contactrwaist(i); }
    }

  


  sotDEBUGOUT(15);
  return res;
}



/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
void WaistPoseFromSensorAndContact::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

  if( cmdLine == "help" )
    {
      os <<"WaistPoseFromSensorAndContact:"<<std::endl
	 <<"  - fromSensor [true|false]: get/set the flag." << std::endl;
      WaistAttitudeFromSensor::commandLine(cmdLine,cmdArgs,os);
    }
  else if( cmdLine == "fromSensor" )
    {
      std::string val; cmdArgs>>val; 
      if( ("true"==val)||("false"==val) )
	{
	  fromSensor = ( val=="true" ); 
	} else {
	  os << "fromSensor = " << (fromSensor?"true":"false") << std::endl;
	}
    }
  else { WaistAttitudeFromSensor::commandLine( cmdLine,cmdArgs,os); }
}

