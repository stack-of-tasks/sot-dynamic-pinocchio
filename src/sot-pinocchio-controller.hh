/*
 * Copyright 2018,  LAAS, CNRS
 *
 * Olivier Stasse
 *
 *
 * See LICENSE.txt.
 */

#ifndef _SOT_PinocchioController_H_
#define _SOT_PinocchioController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

#include "sot-dynamic-pinocchio/pinocchio-device.hh"
#include <dynamic_graph_bridge/ros_interpreter.hh>
namespace dgsot=dynamicgraph::sot;

namespace dynamicgraph {
  namespace sot {
    
    class SoTPinocchioController: public 
    dgsot::AbstractSotExternalInterface
    {
    public:
      
      static const std::string LOG_PYTHON;
      
      SoTPinocchioController();
      SoTPinocchioController(const char robotName[]);
      SoTPinocchioController(std::string robotName);
      virtual ~SoTPinocchioController();
      
      void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
      
      void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
      
      void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);
      
      void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);
      
      void setNoIntegration(void);
      void setSecondOrderIntegration(void);
      
      /// Embedded python interpreter accessible via Corba/ros
      boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;
      
      boost::shared_ptr<ros::NodeHandle> nh_;
      boost::shared_ptr<ros::AsyncSpinner> spinner_;
      
    protected:
      // Update output port with the control computed from the
      // dynamic graph.
      void updateRobotState(std::vector<double> &anglesIn);
      
      /// Run a python command 
      void runPython(std::ostream& file,
		     const std::string& command,
		     dynamicgraph::Interpreter& interpreter);
      
      virtual void startupPython();
      
      void init();
      
      dgsot::PinocchioDevice* device_;
    };
  }
}

#endif /* _SOT_PinocchioController_H_ */

