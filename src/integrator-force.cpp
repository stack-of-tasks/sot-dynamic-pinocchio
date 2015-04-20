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

#include <sot-dynamic/integrator-force.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(IntegratorForce,"IntegratorForce");

const double IntegratorForce:: TIME_STEP_DEFAULT = 5e-3;

IntegratorForce::
IntegratorForce( const std::string & name ) 
  :Entity(name)
   ,timeStep( TIME_STEP_DEFAULT )
   ,forceSIN(NULL,"sotIntegratorForce("+name+")::input(vector)::force")
   ,massInverseSIN(NULL,"sotIntegratorForce("+name+")::input(matrix)::massInverse")
   ,frictionSIN(NULL,"sotIntegratorForce("+name+")::input(matrix)::friction")
   ,velocityPrecSIN(NULL,"sotIntegratorForce("+name+")::input(matrix)::vprec")
  
   ,velocityDerivativeSOUT
  ( boost::bind(&IntegratorForce::computeDerivative,this,_1,_2),
    velocityPrecSIN<<forceSIN<<massInverseSIN<<frictionSIN,
    "sotIntegratorForce("+name+")::output(Vector)::velocityDerivative" )
   ,velocitySOUT( boost::bind(&IntegratorForce::computeIntegral,this,_1,_2),
		 velocityPrecSIN<<velocityDerivativeSOUT,
		 "sotIntegratorForce("+name+")::output(Vector)::velocity" )

   ,massSIN(NULL,"sotIntegratorForce("+name+")::input(matrix)::mass")
  ,massInverseSOUT( boost::bind(&IntegratorForce::computeMassInverse,this,_1,_2),
		    massSIN,
		    "sotIntegratorForce("+name+")::input(matrix)::massInverseOUT")
{
  sotDEBUGIN(5);
  
  signalRegistration(forceSIN);
  signalRegistration(massInverseSIN);
  signalRegistration(frictionSIN);
  signalRegistration(velocityPrecSIN);
  signalRegistration(velocityDerivativeSOUT  );
  signalRegistration(velocitySOUT );
  signalRegistration(massInverseSOUT );
  signalRegistration(massSIN );

  massInverseSIN.plug( &massInverseSOUT );

  sotDEBUGOUT(5);
}


IntegratorForce::
~IntegratorForce( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

/* The derivative of the signal is such that: M v_dot + B v = f. We deduce:
 * v_dot =  M^-1 (f - Bv)
 */
ml::Vector& IntegratorForce::
computeDerivative( ml::Vector& res,
		   const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector & force = forceSIN( time );
  const ml::Matrix & massInverse = massInverseSIN( time );
  const ml::Matrix & friction = frictionSIN( time );

  sotDEBUG(15) << "force = " << force << std::endl;

  ml::Vector f_bv( force.size() );

  if( velocityPrecSIN )
    { 
      const ml::Vector & vel = velocityPrecSIN( time );
      sotDEBUG(15) << "vel = " << vel << std::endl;
      friction .multiply( vel,f_bv ); f_bv *= -1; 
    } else { f_bv.fill(0) ; } // vel is not set yet.
      
  f_bv+=force;
  massInverse.multiply( f_bv, res );
  
  sotDEBUGOUT(15);
  return res;
}

ml::Vector& IntegratorForce::
computeIntegral( ml::Vector& res,
		 const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector & dvel =velocityDerivativeSOUT( time );
  res = dvel;
  res *= timeStep;
  if( velocityPrecSIN )
    {
      const ml::Vector & vel = velocityPrecSIN( time );
      res += vel; 
    } 
  else { /* vprec not set yet. */ }
  velocityPrecSIN = res ;

  sotDEBUGOUT(15);
  return res;
}

ml::Matrix& IntegratorForce::
computeMassInverse( ml::Matrix& res,
		    const int& time )
{
  sotDEBUGIN(15);

  const ml::Matrix & mass = massSIN( time );
  mass.inverse( res );

  sotDEBUGOUT(15);
  return res;
}


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
void IntegratorForce::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

  if( cmdLine == "help" )
    {
      os << "IntegratorForce: " 
	 << "  - dt [<time>]: get/set the timestep value. " << std::endl;
    }
  else if( cmdLine == "dt" )
    {
      cmdArgs >>std::ws; 
      if( cmdArgs.good() )
	{
	  double val; cmdArgs>>val; 
	  if( val >0. ) timeStep = val;
	}
      else { os << "TimeStep = " << timeStep << std::endl;}
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}

