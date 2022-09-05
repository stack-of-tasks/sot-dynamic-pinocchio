/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/LAAS
 *
 */

#ifndef ANGLE_ESTIMATOR_COMMAND_H
#define ANGLE_ESTIMATOR_COMMAND_H

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
namespace sot {
namespace command {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command FromSensor
class FromSensor : public Command {
 public:
  virtual ~FromSensor() {
    sotDEBUGIN(15);
    sotDEBUGOUT(15);
  }
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  FromSensor(Dynamic& entity, const std::string& docstring)
      : Command(entity,
                boost::assign::list_of(Value::STRING)(Value::STRING)(
                    Value::STRING)(Value::STRING),
                docstring) {}
  virtual Value doExecute() {
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
};  // class FromSensor
}  // namespace command
}  // namespace sot
}  // namespace dynamicgraph

#endif  // ANGLE_ESTIMATOR_COMMAND_H
