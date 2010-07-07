/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      IntegratorForceRK4.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot-dynamic/integrator-force-rk4.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>


using namespace sot;
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

ml::Vector& IntegratorForceRK4::
computeDerivativeRK4( ml::Vector& res,
		      const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector & force = forceSIN( time );
  const ml::Matrix & massInverse = massInverseSIN( time );
  const ml::Matrix & friction = frictionSIN( time );
  unsigned int nf = force.size(), nv = friction.nbCols();
  res.resize(nv); res.fill(0);

  if(! velocityPrecSIN )
    { 
      ml::Vector zero( nv ); zero.fill(0);
      velocityPrecSIN = zero;
    } 
  const ml::Vector & vel = velocityPrecSIN( time );
  double & dt = this->IntegratorForce::timeStep; // this is &

  sotDEBUG(15) << "force = " << force;
  sotDEBUG(15) << "vel = " << vel;
  sotDEBUG(25) << "Mi = " << massInverse;
  sotDEBUG(25) << "B = " << friction;
  sotDEBUG(25) << "dt = " << dt << std::endl;

  std::vector<ml::Vector> v(4);
  ml::Vector ki( nv ), fi( nf );
  double sumFact = 0;
  v[0]=vel;

  for( unsigned int i=0;i<4;++i )
    {
      sotDEBUG(35) << "v"<<i<<" = " << v[i];
      friction.multiply( v[i],fi ); fi*=-1;
      fi += force;
      sotDEBUG(35) << "f"<<i<<" = " << fi;
      massInverse.multiply( fi,ki );
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

