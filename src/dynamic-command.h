/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
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

#ifndef DYNAMIC_COMMAND_H
 #define DYNAMIC_COMMAND_H

 #include <fstream>
 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace dynamicgraph { namespace sot {
  namespace command {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    // Command DisplayModel
    class DisplayModel : public Command
    {
    public:
      virtual ~DisplayModel() {	sotDEBUGIN(15);
	sotDEBUGOUT(15);}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      DisplayModel(Dynamic& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	robot.displayModel();
	return Value();
      }
    }; // class DisplayModel


    // Command GetDimension
    class GetDimension : public Command
    {
    public:
      virtual ~GetDimension() {	sotDEBUGIN(15);
	sotDEBUGOUT(15);}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      GetDimension(Dynamic& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	unsigned int dimension = robot.m_model->nv;
	return Value(dimension);
      }
    }; // class GetDimension

  } // namespace command
} /* namespace sot */} /* namespace dynamicgraph */

#endif //DYNAMIC_COMMAND_H
