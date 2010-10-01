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

