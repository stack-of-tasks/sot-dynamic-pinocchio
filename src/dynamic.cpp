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

#include <sot-dynamic-pinocchio/dynamic-pinocchio.h>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

extern "C" {
  ::dynamicgraph::Entity*						
  EntityMaker_Dynamic(const std::string& objname)			
  {									
    return new DynamicPinocchio (objname);					
  }									
  ::dynamicgraph::EntityRegisterer					
  reg_Dynamic ("DynamicPinocchio",						
	       &EntityMaker_Dynamic);				
}
//DYNAMICGRAPH_FACTORY_DYNAMIC_PLUGIN(Dynamic,"Dynamic");
