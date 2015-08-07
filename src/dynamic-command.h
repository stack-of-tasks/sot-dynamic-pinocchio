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
    // Work done
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
    std::string urdfPath = values[0].value();
    robot.setUrdfPath(urdfPath);
	return Value();
      }
    }; // class SetFiles

    // Command Parse
    // Work done
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
    if(! robot.init) std::cout << "No file parsed, run command setFiles" << std::endl;
    else std::cout << "Already parsed!" << std::endl;
    return Value();
      }
    }; // class Parse

    // Command SetProperty
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string property = values[0].value();
//	std::string value = values[1].value();

//	robot.m_HDR->setProperty(property, value);
	return Value();
      }
    }; // class SetProperty

    // Command GetProperty
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string property = values[0].value();
//	std::string value;

//	if(! robot.m_HDR->getProperty(property, value) )
//	  {
//	    if( property == "vrmlDirectory" ) value = robot.vrmlDirectory;
//	    else if( property == "xmlSpecificityFile" ) value = robot.xmlSpecificityFile;
//	    else if( property == "xmlRankFile" ) value = robot.xmlRankFile;
//	    else if( property == "vrmlMainFile" ) value = robot.vrmlMainFile;
//	  }

    return Value(/*value*/);
      }
    }; // class GetProperty

    // Command CreateRobot
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	robot.createRobot();
	return Value();
      }
    }; // class CreateRobot

    // Command CreateJoint
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	std::string jointType = values[1].value();
//	maal::boost::Matrix position = values[2].value();
//	robot.createJoint(jointName, jointType, position);
	return Value();
      }
    }; // class CreateJoint

    // Command SetRootJoint
    // TODO :Implement
    class SetRootJoint : public Command
    {
    public:
      virtual ~SetRootJoint() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetRootJoint(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	robot.setRootJoint(jointName);
	return Value();
      }
    }; // class SetRootJoint

    // Command AddJoint
    // TODO :Implement
    class AddJoint : public Command
    {
    public:
      virtual ~AddJoint() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      AddJoint(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string parentName = values[0].value();
//	std::string childName = values[1].value();
//	robot.addJoint(parentName, childName);
	return Value();
      }
    }; // class AddJoint

    // Command SetDofBounds
    // TODO :Implement
    class SetDofBounds : public Command
    {
    public:
      virtual ~SetDofBounds() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetDofBounds(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::UNSIGNED)
		(Value::DOUBLE)(Value::DOUBLE), docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	unsigned int dofId = values[1].value();
//	double minValue = values[2].value();
//	double maxValue = values[3].value();
//	robot.setDofBounds(jointName, dofId, minValue, maxValue);
	return Value();
      }
    }; // class SetDofBounds

    // Command SetMass
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	double mass = values[1].value();
//	robot.setMass(jointName, mass);
	return Value();
      }
    }; // class SetMass

    // Command SetLocalCenterOfMass
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	ml::Vector com = values[1].value();
//	robot.setLocalCenterOfMass(jointName, com);
	return Value();
      }
    }; // class SetLocalCenterOfMass

    // Command SetInertiaMatrix
    // TODO :Implement
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	ml::Matrix inertiaMatrix = values[1].value();
//	robot.setInertiaMatrix(jointName, inertiaMatrix);
	return Value();
      }
    }; // class SetInertiaMatrix

    // Command SetSpecificJoint
    // TODO :Implement
    class SetSpecificJoint : public Command
    {
    public:
      virtual ~SetSpecificJoint() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetSpecificJoint(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING)(Value::STRING),
		docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string jointName = values[0].value();
//	std::string jointType = values[1].value();
//	robot.setSpecificJoint(jointName, jointType);
	return Value();
      }
    }; // class SetSpecificJoint

    // Command SetHandParameters
    // TODO :Implement
    class SetHandParameters : public Command
    {
    public:
      virtual ~SetHandParameters() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetHandParameters(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::BOOL)(Value::VECTOR)
		(Value::VECTOR)(Value::VECTOR)(Value::VECTOR), docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	bool right = values[0].value();
//	ml::Vector center = values[1].value();
//	ml::Vector thumbAxis = values[2].value();
//	ml::Vector forefingerAxis = values[3].value();
//	ml::Vector palmNormalAxis = values[4].value();
//	robot.setHandParameters(right, center, thumbAxis, forefingerAxis,
//				palmNormalAxis);
	return Value();
      }
    }; // class SetHandParameters

    // Command SetFootParameters
    // TODO :Implement
    class SetFootParameters : public Command
    {
    public:
      virtual ~SetFootParameters() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetFootParameters(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::BOOL)(Value::DOUBLE)
		(Value::DOUBLE)(Value::VECTOR), docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	bool right = values[0].value();
//	double soleLength = values[1].value();
//	double soleWidth = values[2].value();
//	ml::Vector anklePosition = values[3].value();
//	robot.setFootParameters(right, soleLength, soleWidth, anklePosition);
	return Value();
      }
    }; // class Setfootparameters

    // Command SetGazeParameters
    // TODO :Implement
    class SetGazeParameters : public Command
    {
    public:
      virtual ~SetGazeParameters() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      SetGazeParameters(Dynamic& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::VECTOR)(Value::VECTOR),
		docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	ml::Vector gazeOrigin  = values[0].value();
//	ml::Vector gazeDirection = values[1].value();
//	robot.setGazeParameters(gazeOrigin, gazeDirection);
	return Value();
      }
    }; // class SetGazeParameters

    // Command InitializeRobot
    // TODO :Implement
    class InitializeRobot : public Command
    {
    public:
      virtual ~InitializeRobot() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      InitializeRobot(Dynamic& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(),
		docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	robot.m_HDR->initialize();
	return Value();
      }
    }; // class InitializeRobot

    // Command GetDimension
    // TODO :Implement
    class GetDimension : public Command
    {
    public:
      virtual ~GetDimension() {}
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
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	unsigned int dimension = robot.m_HDR->numberDof();
    return Value(/*dimension*/);
      }
    }; // class GetDimension

    // Command Write
    // TODO :Implement
    class Write : public Command
    {
    public:
      virtual ~Write() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      Write(Dynamic& entity, const std::string& docstring) :
      Command(entity, boost::assign::list_of(Value::STRING),
	      docstring)
      {
      }
      virtual Value doExecute()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	std::string filename = values[0].value();
//	std::ofstream file(filename.c_str(), std::ios_base::out);
//	file << *(robot.m_HDR);
//	file.close();
	return Value();
      }
    }; // class Write

    // Command GetHandParameter
    // TODO :Implement
    class GetHandParameter : public Command
    {
    public:
      virtual ~GetHandParameter () {}
      GetHandParameter (Dynamic& entity, const std::string& docstring) :
      Command (entity, boost::assign::list_of(Value::BOOL), docstring)
      {
      }
      virtual Value doExecute ()
      {
//	Dynamic& robot = static_cast<Dynamic&>(owner());
//	std::vector<Value> values = getParameterValues();
//	bool right = values [0].value ();
//	ml::Matrix handParameter (4,4);
//	handParameter.setIdentity ();
//	CjrlHand* hand;
//	if (right) hand = robot.m_HDR->rightHand ();
//	else hand = robot.m_HDR->leftHand ();
//	vector3d axis;
//	hand->getThumbAxis (axis);
//	for (unsigned int i=0; i<3; i++)
//	  handParameter (i,0) = axis (i);
//	hand->getForeFingerAxis (axis);
//	for (unsigned int i=0; i<3; i++)
//	  handParameter (i,1) = axis (i);
//	hand->getPalmNormal (axis);
//	for (unsigned int i=0; i<3; i++)
//	  handParameter (i,2) = axis (i);
//	hand->getCenter (axis);
//	for (unsigned int i=0; i<3; i++)
//	  handParameter (i,3) = axis (i);
    return Value (/*handParameter*/);
      }
    }; // class GetHandParameter
  } // namespace command
} /* namespace sot */} /* namespace dynamicgraph */

#endif //DYNAMIC_COMMAND_H
