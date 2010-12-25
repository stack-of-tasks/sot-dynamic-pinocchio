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

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace sot {
  namespace command {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;
  
    // Command SetFiles
    class SetFiles : public Command
    {
    public:
      virtual ~SetFiles() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetFiles(Dynamic& entity, const std::string& docstring) :
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
    }; // class SetFiles

    // Command Parse
    class Parse : public Command
    {
    public:
      virtual ~Parse() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      Parse(Dynamic& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(), docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	if(! robot.init ) robot.parseConfigFiles();
	else std::cout << "  !! Already parsed." << std::endl;
	// return void
	return Value();
      }
    }; // class Parse

    // Command CreateOpPoint
    class CreateOpPoint : public Command
    {
    public:
      virtual ~CreateOpPoint() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      CreateOpPoint(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string opPointName = values[0].value();
	std::string jointName = values[1].value();
	CjrlJoint* joint = NULL;
	if (jointName ==  "gaze") {
	  joint = robot.m_HDR->gazeJoint();
	} else if (jointName == "left-ankle") {
	  joint = robot.m_HDR->leftAnkle();
	} else if (jointName == "right-ankle") {
	  joint = robot.m_HDR->rightAnkle();
	} else if (jointName == "left-wrist") {
	  joint = robot.m_HDR->leftWrist();
	} else if (jointName == "right-wrist") {
	  joint = robot.m_HDR->rightWrist();
	} else if (jointName == "waist") {
	  joint = robot.m_HDR->waist();
	} else if (jointName == "chest") {
	  joint = robot.m_HDR->chest();
	} else if (jointName == "gaze") {
	  joint = robot.m_HDR->gazeJoint();
	} else {
	  throw ExceptionDynamic(ExceptionDynamic::GENERIC,
				 jointName + " is not a valid name."
				 " Valid names are \n"
				 "gaze, left-ankle, right ankle, left-wrist,"
				 " right-wrist, waist, chest.");
	}
	robot.createEndeffJacobianSignal(std::string("J")+opPointName, joint);
	robot.createPositionSignal(opPointName, joint);
	return Value();
      }
    }; // class CreateOpPoint

    // Command SetProperty
    class SetProperty : public Command
    {
    public:
      virtual ~SetProperty() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetProperty(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string property = values[0].value();
	std::string value = values[1].value();
	
	robot.m_HDR->setProperty(property, value);
	return Value();
      }
    }; // class SetProperty

    // Command GetProperty
    class GetProperty : public Command
    {
    public:
      virtual ~GetProperty() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      GetProperty(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
	Dynamic& robot = static_cast<Dynamic&>(owner());
	std::vector<Value> values = getParameterValues();
	std::string property = values[0].value();
	std::string value;

	robot.m_HDR->getProperty(property, value);
	return Value(value);
      }
    }; // class GetProperty

  } // namespace command
} //namespace sot

#endif //DYNAMIC_COMMAND_H
