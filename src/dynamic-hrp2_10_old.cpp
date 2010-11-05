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

#include <sot-dynamic/dynamic-hrp2_10_old.h>
#include <sot-core/debug.h>

#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

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

