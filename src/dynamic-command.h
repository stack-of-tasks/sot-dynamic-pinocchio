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

    // Command SetFiles
    class SetFile : public Command
    {
    public:
      virtual ~SetFile() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetFile(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string urdfFile = values[0].value();
	robot.setUrdfFile(urdfFile);
	return Value();
      }
    }; // class SetFiles

    // Command CreateRobot
    class CreateRobot : public Command
    {
    public:
      virtual ~CreateRobot() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      CreateRobot(Dynamic& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	robot.createRobot();
	return Value();
      }
    }; // class CreateRobot

    // Command DisplayModel
    class DisplayModel : public Command
    {
    public:
      virtual ~DisplayModel() {}
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

    // Command CreateJoint
    class CreateJoint : public Command
    {
    public:
      virtual ~CreateJoint() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      CreateJoint(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING)
		(Value::MATRIX), docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	std::string jointType = values[1].value();
	dynamicgraph::Matrix position = values[2].value();
	robot.createJoint(jointName, jointType, position);
	return Value();
      }
    }; // class CreateJoint

    // Command AddBody
    class AddBody : public Command
    {
    public:
      virtual ~AddBody() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      AddBody(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string parentName = values[0].value();
	std::string jointName = values[1].value();
	std::string childName = values[2].value();
	robot.addBody(parentName,jointName,childName);
	return Value();
      }
    }; // class AddBody

    // Command SetMass
    class SetMass : public Command
    {
    public:
      virtual ~SetMass() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetMass(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::DOUBLE),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	double mass = values[1].value();
	robot.setMass(jointName, mass);
	return Value();
      }
    }; // class SetMass

    // Command SetLocalCenterOfMass
    class SetLocalCenterOfMass : public Command
    {
    public:
      virtual ~SetLocalCenterOfMass() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetLocalCenterOfMass(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dynamicgraph::Vector com = values[1].value();
	robot.setLocalCenterOfMass(jointName, com);
	return Value();
      }
    }; // class SetLocalCenterOfMass

    // Command SetInertiaMatrix
    class SetInertiaMatrix : public Command
    {
    public:
      virtual ~SetInertiaMatrix() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetInertiaMatrix(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::MATRIX),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dynamicgraph::Matrix inertiaMatrix = values[1].value();
	robot.setInertiaMatrix(jointName, inertiaMatrix);
	return Value();
      }
    }; // class SetInertiaMatrix

    // Command SetInertiaProperties
    class SetInertiaProperties : public Command
    {
    public:
      virtual ~SetInertiaProperties() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetInertiaProperties(Dynamic& entity, const std::string& docstring) :
	Command(entity, 
		boost::assign::list_of(Value::STRING)
		(Value::DOUBLE)(Value::VECTOR)(Value::MATRIX),docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	double mass = values[1].value();
	dynamicgraph::Vector com = values[2].value();
	dynamicgraph::Matrix inertiaMatrix = values[3].value();
	robot.setInertiaProperties(jointName, mass,com,inertiaMatrix);
	return Value();
      }
    }; // class SetInertiaMatrix


    // Command SetLowerPositionLimit
    class SetLowerPositionLimit : public Command
    {
    public:
      virtual ~SetLowerPositionLimit() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetLowerPositionLimit(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dg::Vector in_vector = values[1].value();
	robot.setLowerPositionLimit(jointName,in_vector);
	return Value();
      }
    }; // class SetLowerPositionLimit

    // Command SetUpperPositionLimit
    class SetUpperPositionLimit : public Command
    {
    public:
      virtual ~SetUpperPositionLimit() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetUpperPositionLimit(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dg::Vector in_vector = values[1].value();
	robot.setUpperPositionLimit(jointName,in_vector);
	return Value();
      }
    }; // class SetUpperPositionLimit

    // Command SetMaxVelocityLimit
    class SetMaxVelocityLimit : public Command
    {
    public:
      virtual ~SetMaxVelocityLimit() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetMaxVelocityLimit(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dg::Vector in_vector = values[1].value();
	robot.setMaxVelocityLimit(jointName,in_vector);
	return Value();
      }
    }; // class SetMaxVelocityLimit


    // Command SetMaxEffortLimit
    class SetMaxEffortLimit : public Command
    {
    public:
      virtual ~SetMaxEffortLimit() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetMaxEffortLimit(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string jointName = values[0].value();
	dg::Vector in_vector = values[1].value();
	robot.setMaxEffortLimit(jointName,in_vector);
	return Value();
      }
    }; // class SetMaxEffortLimit



  } // namespace command
} /* namespace sot */} /* namespace dynamicgraph */

#endif //DYNAMIC_COMMAND_H
