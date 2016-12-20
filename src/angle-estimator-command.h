/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/LAAS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ANGLE_ESTIMATOR_COMMAND_H
 #define ANGLE_ESTIMATOR_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace dynamicgraph { namespace sot {
  namespace command {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;
  
    // Command FromSensor
    class FromSensor : public Command
    {
    public:
      virtual ~FromSensor() {
	sotDEBUGIN(15);
	sotDEBUGOUT(15);
      }
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      FromSensor(Dynamic& entity, const std::string& docstring) :
      Command(entity, boost::assign::list_of(Value::STRING)
	      (Value::STRING)(Value::STRING)(Value::STRING), docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string vrmlDirectory = values[0].value();
	std::string vrmlMainFile = values[1].value();
	std::string xmlSpecificityFiles = values[2].value();
	std::string xmlRankFile = values[3].value();
	robot.setVrmlDirectory(vrmlDirectory);
	robot.setVrmlMainFile(vrmlMainFile);
	robot.setXmlSpecificityFile(xmlSpecificityFiles);
	robot.setXmlRankFile(xmlRankFile);
	// return void
	return Value();
      }
    }; // class FromSensor
  } // namespace command
  } // namespace sot
} // namespace dynamicgraph

#endif //ANGLE_ESTIMATOR_COMMAND_H
