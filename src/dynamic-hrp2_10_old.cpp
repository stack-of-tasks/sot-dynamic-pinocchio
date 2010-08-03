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

#include <sot-dynamic/dynamic-hrp2_10_old.h>
#include <sot-core/debug.h>

#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>

#include <dynamic-graph/factory.h>
using namespace dynamicgraph;
using namespace sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicHrp2_10_old,"DynamicHrp2_10_old");

DynamicHrp2_10_old::
DynamicHrp2_10_old( const std::string & name ) 
  :Dynamic(name,false)
{
  sotDEBUGIN(15);
  DynamicHrp2_10_old::buildModelHrp2();
  sotDEBUGOUT(15);
}

void DynamicHrp2_10_old::
buildModelHrp2( void )
{
  sotDEBUGIN(15);
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  
  hrp210Optimized<paramHrp210SmallOldOptimized> * aHDR = 
    new hrp210Optimized<paramHrp210SmallOldOptimized>(&aRobotDynamicsObjectConstructor);

  m_HDR = aHDR;

  if (aHDR==0)
    { 
      std::cerr<< "Dynamic cast on HDR failed " << std::endl;
      exit(-1);
    }

  sotDEBUGOUT(15);
}


DynamicHrp2_10_old::
~DynamicHrp2_10_old( void )
{
  sotDEBUGINOUT(5);
  return;
}

