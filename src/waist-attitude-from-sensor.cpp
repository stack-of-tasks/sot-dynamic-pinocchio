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
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

using namespace dynamicgraph::sot;
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

  signalRegistration(attitudeSensorSIN);
  signalRegistration(positionSensorSIN);
  signalRegistration(attitudeWaistSOUT);

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
   ,fromSensor_(false)
   ,positionContactSIN(NULL,"sotWaistPoseFromSensorAndContact("+name+")::input(matrixHomo)::contact")
   ,positionWaistSOUT( boost::bind(&WaistPoseFromSensorAndContact::computePositionWaist,this,_1,_2),
		       attitudeWaistSOUT<<positionContactSIN,
		       "sotWaistPoseFromSensorAndContact("+name+")::output(RPY+T)::positionWaist" ) 
{
  sotDEBUGIN(5);

  signalRegistration( positionContactSIN);
  signalRegistration(positionWaistSOUT );

  // Commands
  std::string docstring;
  docstring = "    \n"
    "    Set flag specifying whether angles are measured from sensors or simulated.\n"
    "    \n"
    "      Input:\n"
    "        - a boolean value.\n"
    "    \n";
  addCommand("setFromSensor",
	     new ::dynamicgraph::command::Setter
	     <WaistPoseFromSensorAndContact, bool>
	     (*this, &WaistPoseFromSensorAndContact::fromSensor, docstring));

    "    Get flag specifying whether angles are measured from sensors or simulated.\n"
    "    \n"
    "      No input,\n"
    "      return a boolean value.\n"
    "    \n";
  addCommand("getFromSensor",
	     new ::dynamicgraph::command::Getter
	     <WaistPoseFromSensorAndContact, bool>
	     (*this, &WaistPoseFromSensorAndContact::fromSensor, docstring));

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

  if(fromSensor_)
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
	  fromSensor_ = ( val=="true" ); 
	} else {
	  os << "fromSensor = " << (fromSensor_?"true":"false") << std::endl;
	}
    }
  else { WaistAttitudeFromSensor::commandLine( cmdLine,cmdArgs,os); }
}

