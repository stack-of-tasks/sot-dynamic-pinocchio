/* 
 * Copyright 2018, LAAS-CNRS
 *
 * Olivier Stasse
 *

 * See LICENSE.txt
 *
 */

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

#include "sot-dynamic-pinocchio/sot-pinocchio-controller.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <ros/console.h>
using namespace dynamicgraph::sot;

const std::string SoTPinocchioController::LOG_PYTHON="/tmp/PinocchioController_python.out";

using namespace std;

SoTPinocchioController::SoTPinocchioController(std::string RobotName):
  device_(new PinocchioDevice(RobotName))
{
  init();
}

SoTPinocchioController::SoTPinocchioController(const char robotName[]):
  device_(new PinocchioDevice(robotName))
{
  init();
}

void SoTPinocchioController::
readActuator(ros::NodeHandle &nh,
	     std::string &actuator_name)
{
  std::string aparameterName("/map_hardware_sot_control/"+actuator_name);
  if (nh.hasParam(aparameterName))
    {
      // Getting HW Control type.
      std::string param_title=aparameterName+"/hw";
      std::string param_value;
      if (nh.hasParam(param_title))
	{
	  nh.getParam(param_title,param_value);
	  device_->setHWControlType(actuator_name,param_value);
	}
      
      param_title=aparameterName+"/sot";
      if (nh.hasParam(param_title))
	{
	  nh.getParam(param_title,param_value);
	  device_->setSoTControlType(actuator_name,param_value);
	}

      param_title=aparameterName+"/controlPos";
      double aDouble;
      if (nh.hasParam(param_title))
	{
	  nh.getParam(param_title,aDouble);
	  unsigned int index = (unsigned int) aDouble;
	  device_->setControlPos(actuator_name,index);
	}

    }
}
void SoTPinocchioController::init()
{
  std::cout << "Going through SoTPinocchioController." << std::endl;

  // rosInit is called here only to initialize ros.
  // No spinner is initialized.
  ros::NodeHandle &nh = dynamicgraph::rosInit(false,false);

  /// Reading the robot description
  std::string robot_description;
  if (nh.hasParam("/robot_description"))
    {
      nh.getParam("/robot_description",robot_description);
      device_->setURDFModel(robot_description);
      std::vector<std::string> alist_of_actuators;
      
      if (nh.getParam("/map_hardware_sot_control/",
		      alist_of_actuators))
	{
	  for(unsigned int i=0;i<alist_of_actuators.size();i++)
	    {
	      std::cout << alist_of_actuators[i] << std::endl;
	    }
	}

      
	  
    }
      
  interpreter_ = boost::shared_ptr<dynamicgraph::Interpreter>(
      new dynamicgraph::Interpreter (nh));

  sotDEBUG(25) << __FILE__ << ":" 
	       << __FUNCTION__ <<"(#" 
	       << __LINE__ << " )" << std::endl;

  double ts = ros::param::param<double> ("/sot_controller/dt",
					 PinocchioDevice::TIMESTEP_DEFAULT);
  device_->timeStep(ts);
}

SoTPinocchioController::~SoTPinocchioController()
{
  // device_ will be deleted by dynamicgraph::PoolStorage::destroy()
}

void SoTPinocchioController::
setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_->setupSetSensors(SensorsIn);
}


void SoTPinocchioController::
nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_->nominalSetSensors(SensorsIn);
}

void SoTPinocchioController::
cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  device_->cleanupSetSensors(SensorsIn);
}


void SoTPinocchioController::
getControl(map<string,dgsot::ControlValues> &controlOut)
{
  try 
    {
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
      device_->getControl(controlOut);
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
    }
  catch ( dynamicgraph::sot::ExceptionAbstract & err)
    {

      std::cout << __FILE__ << " " 
		<< __FUNCTION__ << " (" 
		<< __LINE__ << ") " 
		<< err.getStringMessage() 
		<<  endl;
      throw err;
    }
}

void SoTPinocchioController::
setNoIntegration(void)
{
}

void SoTPinocchioController::
setSecondOrderIntegration(void)
{
}

void SoTPinocchioController::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string lres(""),lout(""),lerr("");
  interpreter.runCommand(command,lres,lout,lerr);

  if (lres != "None")
    {
      if (lres=="<NULL>")
	{
	  file << lout << std::endl;
	  file << "------" << std::endl;
	  file << lerr << std::endl;
	  ROS_INFO(lout.c_str());
	  ROS_ERROR(lerr.c_str());
	}
      else
	{
	  file << lres << std::endl;
	  ROS_INFO(lres.c_str());
	}
    }
}

void SoTPinocchioController::
startupPython()
{
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython (aof, "import sys, os", *interpreter_);
  runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
  runPython (aof, "path = []", *interpreter_);
  runPython (aof,
	     "for p in pythonpath.split(':'):\n"
	     "  if p not in sys.path:\n"
	     "    path.append(p)", *interpreter_);
  runPython (aof, "path.extend(sys.path)", *interpreter_);
  runPython (aof, "sys.path = path", *interpreter_);

  // Calling again rosInit here to start the spinner. It will
  // deal with topics and services callbacks in a separate, non
  // real-time thread. See roscpp documentation for more
  // information.
  dynamicgraph::rosInit (true);
  aof.close();
}

