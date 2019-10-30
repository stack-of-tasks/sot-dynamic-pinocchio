/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 */

#ifndef DYNAMIC_COMMAND_H
#define DYNAMIC_COMMAND_H

#include <fstream>
#include <boost/assign/list_of.hpp>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

namespace dynamicgraph {
namespace sot {
namespace command {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command DisplayModel
class DisplayModel : public Command {
 public:
  virtual ~DisplayModel() {
    sotDEBUGIN(15);
    sotDEBUGOUT(15);
  }
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  DisplayModel(DynamicPinocchio& entity, const std::string& docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    DynamicPinocchio& robot = static_cast<DynamicPinocchio&>(owner());
    robot.displayModel();
    return Value();
  }
};  // class DisplayModel

// Command GetDimension
class GetDimension : public Command {
 public:
  virtual ~GetDimension() {
    sotDEBUGIN(15);
    sotDEBUGOUT(15);
  }
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  GetDimension(DynamicPinocchio& entity, const std::string& docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    DynamicPinocchio& robot = static_cast<DynamicPinocchio&>(owner());
    unsigned int dimension = robot.m_model->nv;
    return Value(dimension);
  }
};  // class GetDimension

}  // namespace command
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // DYNAMIC_COMMAND_H
