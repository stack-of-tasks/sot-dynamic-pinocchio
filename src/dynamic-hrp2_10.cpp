/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright JRL-Japan, 2010
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      DynamicHrp2_10.h
 * Project:   SOT
 * Author:    Olivier Stasse
 *            Nicolas Mansard
 *
 * For license see license.txt
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot-dynamic/dynamic-hrp2_10.h>
#include <sot-core/debug.h>

#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>

#include <dynamic-graph/factory.h>
using namespace dynamicgraph;
using namespace sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicHrp2_10,"DynamicHrp2_10");


DynamicHrp2_10::
DynamicHrp2_10( const std::string & name ) 
  :Dynamic(name,false)
{
  sotDEBUGIN(15);
  DynamicHrp2_10::buildModelHrp2();
  sotDEBUGOUT(15);
}

void DynamicHrp2_10::
buildModelHrp2( void )
{
  sotDEBUGIN(15);
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  
  hrp210Optimized<paramHrp210SmallOptimized> * aHDR = 
    new hrp210Optimized<paramHrp210SmallOptimized>(&aRobotDynamicsObjectConstructor);

  m_HDR = aHDR;

  if (aHDR==0)
    { 
      std::cerr<< "Dynamic cast on HDR failed " << std::endl;
      exit(-1);
    }

  sotDEBUGOUT(15);
}


DynamicHrp2_10::
~DynamicHrp2_10( void )
{
  sotDEBUGINOUT(5);
  return;
}

