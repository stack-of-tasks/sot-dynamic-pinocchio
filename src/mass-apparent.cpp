/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      MassApparent.h
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

#include <sot-dynamic/mass-apparent.h>
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MassApparent,"MassApparent");


MassApparent::
MassApparent( const std::string & name ) 
  :Entity(name)
   ,jacobianSIN(NULL,"sotMassApparent("+name+")::input(vector)::jacobian")
   ,inertiaInverseSIN(NULL,"sotMassApparent("+name+")::input(vector)::inertiaInverse")
   ,massInverseSOUT( boost::bind(&MassApparent::computeMassInverse,this,_1,_2),
		     jacobianSIN<<inertiaInverseSIN,
		    "sotMassApparent("+name+")::output(Vector)::massInverse" )
   ,massSOUT( boost::bind(&MassApparent::computeMass,this,_1,_2),
	      massInverseSOUT,
	      "sotMassApparent("+name+")::output(Vector)::mass" )

   ,inertiaSIN(NULL,"sotMassApparent("+name+")::input(vector)::inertia")
  ,inertiaInverseSOUT( boost::bind(&MassApparent::computeInertiaInverse,this,_1,_2),
		       inertiaSIN,
		       "sotMassApparent("+name+")::input(vector)::inertiaInverseOUT")
{
  sotDEBUGIN(5);
  
  signalRegistration( jacobianSIN << inertiaInverseSIN << massInverseSOUT << massSOUT 
		      << inertiaSIN << inertiaInverseSOUT);
  inertiaInverseSIN.plug( &inertiaInverseSOUT );

  sotDEBUGOUT(5);
}


MassApparent::
~MassApparent( void )
{
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
ml::Matrix& MassApparent::
computeMassInverse( ml::Matrix& res,
		   const int& time )
{
  sotDEBUGIN(15);
  
  const ml::Matrix & J = jacobianSIN( time );
  const ml::Matrix & A = inertiaInverseSIN( time );

  ml::Matrix AJt( J.nbCols(),J.nbRows() );
  A.multiply( J.transpose(),AJt );

  res.resize( J.nbRows(),J.nbRows() );
  J.multiply( AJt,res );

  sotDEBUGOUT(15);
  return res;
}

ml::Matrix& MassApparent::
computeMass( ml::Matrix& res,
		   const int& time )
{
  sotDEBUGIN(15);

  const ml::Matrix & omega = massInverseSOUT( time );
  omega.inverse( res );

  sotDEBUGOUT(15);
  return res;
}

ml::Matrix& MassApparent::
computeInertiaInverse( ml::Matrix& res,
		       const int& time )
{
  sotDEBUGIN(15);

  const ml::Matrix & A = inertiaSIN( time );
  A.inverse( res );

  sotDEBUGOUT(15);
  return res;
}
