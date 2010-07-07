/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Dynamic.h
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
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <sot-dynamic/dynamic-hrp2.h>
#include <sot-core/debug.h>

#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>

#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicHrp2,"DynamicHrp2");

DynamicHrp2::
DynamicHrp2( const std::string & name ) 
  :Dynamic(name,false)
{
  sotDEBUGIN(15);
  DynamicHrp2::buildModelHrp2();
  sotDEBUGOUT(15);
}

void DynamicHrp2::
buildModelHrp2( void )
{
  sotDEBUGIN(15);
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  
  Chrp2OptHumanoidDynamicRobot * aHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  
  m_HDR = aHDR;

  if (aHDR==0)
    { 
      std::cerr<< "Dynamic cast on HDR failed " << std::endl;
      exit(-1);
    }

  sotDEBUGOUT(15);
}


DynamicHrp2::
~DynamicHrp2( void )
{
  sotDEBUGINOUT(5);
  return;
}

