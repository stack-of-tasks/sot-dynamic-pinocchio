/*
 * Copyright 2010,
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

#include <sot-dynamic-pinocchio/integrator-force-rk4.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(IntegratorForceRK4,"IntegratorForceRK4");

IntegratorForceRK4::
IntegratorForceRK4( const std::string & name ) 
  :IntegratorForce(name)
{
  sotDEBUGIN(5);
  
  velocityDerivativeSOUT.
    setFunction( boost::bind(&IntegratorForceRK4::computeDerivativeRK4,
			     this,_1,_2));
  
  sotDEBUGOUT(5);
}


IntegratorForceRK4::
~IntegratorForceRK4( void )
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
 * Using RK4 method (doc: wikipedia ;) ): dv= dt/6 ( k1 + 2.k2 + 2.k3 + k4)
 */
static const double rk_fact[4] = { 1.,2.,2.,1. };

dynamicgraph::Vector& IntegratorForceRK4::
computeDerivativeRK4( dynamicgraph::Vector& res,
		      const int& time )
{
  sotDEBUGIN(15);

  const dynamicgraph::Vector & force = forceSIN( time );
  const dynamicgraph::Matrix & massInverse = massInverseSIN( time );
  const dynamicgraph::Matrix & friction = frictionSIN( time );
  long int nf = force.size(), nv = friction.cols();
  res.resize(nv); res.fill(0);

  if(! velocityPrecSIN )
    { 
      dynamicgraph::Vector zero( nv ); zero.fill(0);
      velocityPrecSIN = zero;
    } 
  const dynamicgraph::Vector & vel = velocityPrecSIN( time );
  double & dt = this->IntegratorForce::timeStep; // this is &

  sotDEBUG(15) << "force = " << force;
  sotDEBUG(15) << "vel = " << vel;
  sotDEBUG(25) << "Mi = " << massInverse;
  sotDEBUG(25) << "B = " << friction;
  sotDEBUG(25) << "dt = " << dt << std::endl;

  std::vector<dynamicgraph::Vector> v(4);
  dynamicgraph::Vector ki( nv ), fi( nf );
  double sumFact = 0;
  v[0]=vel;

  for( unsigned int i=0;i<4;++i )
    {
      sotDEBUG(35) << "v"<<i<<" = " << v[i];
      fi = friction*v[i]; fi*=-1;
      fi += force;
      sotDEBUG(35) << "f"<<i<<" = " << fi;
      ki = massInverse*fi;
      sotDEBUG(35) << "k"<<i<<" = " << ki;
      if( i+1<4 ) 
	{
	  v[i+1] = ki;  v[i+1] *= (dt/rk_fact[i+1]);
	  v[i+1] += vel; 
	}
      ki *= rk_fact[i];
      res += ki;
      sotDEBUG(35) << "sum_k"<<i<<" = " << res;
      sumFact += rk_fact[i];
    }
  
  sotDEBUG(35) << "sum_ki = " << res;
  res *= (1/sumFact);
  
  sotDEBUGOUT(15);
  return res;
}


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
// void IntegratorForceRK4::
// commandLine( const std::string& cmdLine,
// 	     std::istringstream& cmdArgs,
// 	     std::ostream& os )
// {
//   sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

//   if( cmdLine == "help" )
//     {
//       os << "IntegratorForceRK4: " << std::endl;
//     }
//   else { IntegratorForce::commandLine( cmdLine,cmdArgs,os); }
// }

