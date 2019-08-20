/*
 * Copyright 2018,
 * Olivier Stasse,
 *
 * CNRS
 * See LICENSE.txt
 *
 */

#include <iostream>
#include <sot/core/debug.hh>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/device.hh>
#include "sot-dynamic-pinocchio/state-integrator.h"

#include <sstream>
#include <fstream>

namespace dg = dynamicgraph;

#define BOOST_TEST_MODULE test-state-integrator

int ReadYAMLFILE(dg::sot::Device &aDevice) {
  // Reflect how the data are splitted in two yaml files in the sot
  std::ifstream yaml_file_controller("../../unitTesting/sot_controller.yaml"); 
  std::string yaml_string_controller;
  yaml_string_controller.assign((std::istreambuf_iterator<char>(yaml_file_controller) ),
                                (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_controller);

  std::ifstream yaml_file_params("../../unitTesting/sot_params.yaml"); 
  std::string yaml_string_params;
  yaml_string_params.assign((std::istreambuf_iterator<char>(yaml_file_params) ),
                            (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_params);
  return 0;
}


int main(int, char **) {

  unsigned int debug_mode = 2;

  std::string robot_description;
  ifstream urdfFile;
  std::string filename = "/opt/openrobots/share/simple_humanoid_description/urdf/simple_humanoid.urdf";
  urdfFile.open(filename.c_str());
  if (!urdfFile.is_open()) {
    std::cerr << "Unable to open " << filename << std::endl;
    return -1;
  }
  stringstream strStream;
  strStream << urdfFile.rdbuf();
  robot_description = strStream.str();

  /// Test reading the URDF file.
  dg::sot::Device aDevice(std::string("simple_humanoid"));
  aDevice.setDebugMode(debug_mode);
  aDevice.setURDFModel(robot_description);

  if (ReadYAMLFILE(aDevice) < 0)
    return -1;

  dg::sot::StateIntegrator aIntegrator(std::string("integrator"));
  aIntegrator.setURDFModel(robot_description);

  dg::Vector aState(29); // without freeFlyer
  for(unsigned j=0;j<aState.size();j++)
    aState(j) = 0.0;
  aIntegrator.setState(aState);

  /// Fix constant vector for the control entry and type
  dg::Vector aControlVector(35); // with freeFlyer
  dg::Vector aControlTypeVector(35); // with freeFlyer
  double dt = 0.005;
  for (unsigned int i = 0; i < 35; i++)
  {
    aControlVector[i] = -0.5;
    aControlTypeVector[i] = 0; //velocity
  }
  aIntegrator.controlSIN.setConstant(aControlVector);
  aIntegrator.controlTypeSIN.setConstant(aControlTypeVector);

  std::cout << "Size of the integrator stateSOUT_ control vector: " << aIntegrator.stateSOUT_(0).size() << std::endl;
  aDevice.controlSIN.plug(&aIntegrator.stateSOUT_);
  for (unsigned int i = 0; i < 2000; i++)
  {
    aIntegrator.integrate(dt);
    aDevice.increment();
  }
  const dg::Vector & poseFF = aIntegrator.freeFlyerPositionOdomSOUT_(2001);
  std::cout << "\n ########### \n " << std::endl;
  std::cout << "Final freeFlyerPositionOdomSOUT_:  " << poseFF << std::endl;

  const dg::sot::MatrixHomogeneous & ffpose = aIntegrator.freeFlyerPose();
  std::cout << "Final freeFlyerPosition MatrixHomogeneous:  " 
  << ffpose.translation() << "\n" 
  << ffpose.linear() << std::endl;

  std::cout << "Final integrator stateSOUT_ :  " << aIntegrator.stateSOUT_(2001) << std::endl;


  const urdf::ModelInterfaceSharedPtr aModel = aDevice.getModel();

  const dg::Vector & aControl = aDevice.motorcontrolSOUT_(2001);
  map<string,dgsot::ControlValues> controlOut;
  aDevice.getControl(controlOut);
  double diff = 0, diffCont = 0, ldiff, ldiffCont;

  vector< ::urdf::JointSharedPtr > urdf_joints = aDevice.getURDFJoints();

  dgsot::JointSHWControlType_iterator it_control_type;
  for (it_control_type  = aDevice.jointDevices_.begin();
       it_control_type != aDevice.jointDevices_.end();
       it_control_type++) 
  {
    int lctl_index = it_control_type->second.control_index;
    int u_index = it_control_type->second.urdf_index;
    std::cout << "\n ########### \n " << std::endl;
    std::cout << "urdf_joints: " << urdf_joints[u_index]->name << std::endl;

    if (it_control_type->second.SoTcontrol == dgsot::POSITION) 
    {
      if (u_index != -1 && (urdf_joints[u_index]->limits)) 
      {        
        double lowerLim = urdf_joints[u_index]->limits->lower;
        ldiff = (aControl[lctl_index] - lowerLim);
        ldiffCont = controlOut["control"].getValues()[lctl_index] - lowerLim;
        diff += ldiff;
        diffCont += ldiffCont;
        std::cout << "Position lowerLim: " << lowerLim << "\n"
                  << "motorcontrolSOUT: " << aControl[lctl_index]  << " -- "
                  << "diff: " << ldiff << "\n" 
                  << "controlOut: " << controlOut["control"].getValues()[lctl_index] << " -- "
                  << "diff: " << ldiffCont << " \n"
                  << "Velocity limit: " << urdf_joints[u_index]->limits->velocity
                  << std::endl;
      }
    }
    else
    {
      std::cout << "motorcontrolSOUT: " << aControl[lctl_index]<< "\n" 
                << "controlOut: " << controlOut["control"].getValues()[lctl_index] << std::endl;
    }
  }
  std::cout << "\n ########### \n " << std::endl;
  std::cout << "totalDiff: " << diff << std::endl;
  std::cout << "totalDiffCont: " << diffCont << std::endl;

  return 0;
}
