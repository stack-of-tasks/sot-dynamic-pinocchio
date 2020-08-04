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
#include <boost/test/unit_test.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/generic-device.hh>
#include "sot/dynamic-pinocchio/state-integrator.h"

#include <sstream>
#include <fstream>

namespace dg = dynamicgraph;

#define BOOST_TEST_MODULE test-state-integrator

int ReadYAMLFILE(dg::sot::GenericDevice &aDevice) {
  // Reflect how the data are splitted in two yaml files in the sot
  std::ifstream yaml_file_controller("sot_controller.yaml");
  std::string yaml_string_controller;
  yaml_string_controller.assign((std::istreambuf_iterator<char>(yaml_file_controller) ),
                                (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_controller);

  std::ifstream yaml_file_params("sot_params.yaml");
  std::string yaml_string_params;
  yaml_string_params.assign((std::istreambuf_iterator<char>(yaml_file_params) ),
                            (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_params);
  return 0;
}

BOOST_AUTO_TEST_CASE(test_state_integrator) {

  unsigned int debug_mode = 2;

  // Get environment variable CMAKE_PREFIX_PATH
  const string s_cmake_prefix_path = getenv( "CMAKE_PREFIX_PATH" );

  // Read the various paths
  vector<string> paths;
  boost::split(paths, s_cmake_prefix_path, boost::is_any_of(":;"));

  // Search simple_humanoid.urdf
  string filename="";
  for (auto test_path : paths)
  {
    filename = test_path +
        string("/share/simple_humanoid_description/urdf/simple_humanoid.urdf");
    if ( boost::filesystem::exists(filename))
      break;
  }

  // If not found fails
  if (filename.size()==0)
  {
    cerr << "Unable to find simple_humanoid_description/urdf/simple_humanoid.urdf in CMAKE_PREFIX_PATH" << endl;
    exit(-1);
  }

  // Otherwise read the file
  ifstream urdfFile(filename);
  ostringstream strStream;
  strStream << urdfFile.rdbuf();
  std::string robot_description;
  robot_description = strStream.str();

  /// Test reading the URDF file.
  dg::sot::GenericDevice aDevice(std::string("simple_humanoid"));
  aDevice.setDebugMode(debug_mode);
  aDevice.setURDFModel(robot_description);

  if (ReadYAMLFILE(aDevice) < 0){
    BOOST_CHECK(false);
    return;
  }
  
  dg::sot::StateIntegrator aIntegrator(std::string("integrator"));
  aIntegrator.init(0.005);
  aIntegrator.setURDFModel(robot_description);

  dg::Vector aState(29); // without freeFlyer
  for (unsigned j = 0; j < aState.size(); j++)
    aState(j) = 0.0;
  aIntegrator.setState(aState);

  /// Fix constant vector for the control entry of the integrator
  dg::Vector aControlVector(29);
  for (unsigned int i = 0; i < 29; i++) {
    aControlVector[i] = -0.5; // in velocity
  }
  aIntegrator.controlSIN.setConstant(aControlVector);

  // Fix FreeFlyer control entry of the integrator
  dg::Vector aFFControlVector(6); //TWIST
  for (unsigned int i = 0; i < 6; i++) {
    aFFControlVector[i] = -0.5; // in velocity
  }
  aIntegrator.freeFlyerSIN.setConstant(aFFControlVector);

  // Set the type vector defining the type of control for each joint
  // With strings
  Eigen::Matrix<std::string, 29, 1> aControlTypeVector;
  for (unsigned int i = 0; i < 29; i++) {
    aControlTypeVector[i] = "qVEL"; //velocity
  }
  aIntegrator.setControlType(aControlTypeVector);
  // Set type of control for the FreeFlyer
  aIntegrator.setControlTypeFreeFlyer("ffVEL"); //in velocity

  // With int -> for addCommand
  // dg::Vector aControlTypeVector(29);
  // Types in int qVEL:0 | qACC:1 | ffVEL:2 | ffACC:3
  // aIntegrator.setControlTypeFreeFlyer("ffVEL"); // with freeFlyer in velocity
  // for (unsigned int i = 0; i < 29; i++)
  // {
  //   aControlTypeVector[i] = 0.0; //velocity
  // }
  // aIntegrator.setControlTypeInt(aControlTypeVector);

  // PLUG the output signal of the integrator to the entry of the device
  aDevice.stateSIN.plug(&aIntegrator.stateSOUT_);
  for (unsigned int i = 0; i < 2000; i++) {
    aDevice.motorcontrolSOUT_.recompute(i);
    aDevice.motorcontrolSOUT_.setReady();
    aIntegrator.stateSOUT_.setReady();
    
  }
  const dg::Vector & poseFF = aIntegrator.freeFlyerPositionEulerSOUT_(2001);
  std::cout << "\n ########### \n " << std::endl;
  std::cout << "Final freeFlyerPositionEulerSOUT_: \n" << poseFF << std::endl;

  std::cout << "\n ########### \n " << std::endl;
  const dg::Vector & poseFFQ = aIntegrator.freeFlyerPositionQuatSOUT_(2001);
  std::cout << "Final freeFlyerPositionQuatSOUT_: \n" << poseFFQ << std::endl;

  std::cout << "\n ########### \n " << std::endl;
  const dg::sot::MatrixHomogeneous & ffposeMat = aIntegrator.freeFlyerPose();
  std::cout << "Final freeFlyerPosition MatrixHomogeneous: \n"
            << ffposeMat.translation() << "\n"
            << ffposeMat.linear() << std::endl;

  std::cout << "\n ########### \n " << std::endl;
  std::cout << "Final integrator stateSOUT_ :  \n" << aIntegrator.stateSOUT_(2001) << std::endl;

  const dg::Vector & aControl = aDevice.motorcontrolSOUT_(2001);
  double diff = 0, ldiff;

  vector< ::urdf::JointSharedPtr > urdf_joints = aDevice.getURDFJoints();

  dgsot::JointSHWControlType_iterator it_control_type;
  for (it_control_type  = aDevice.jointDevices_.begin();
       it_control_type != aDevice.jointDevices_.end();
       it_control_type++) {
    int lctl_index = it_control_type->second.control_index;
    int u_index = it_control_type->second.urdf_index;
    std::cout << "\n ########### \n " << std::endl;
    std::cout << "urdf_joints: " << urdf_joints[u_index]->name << std::endl;

    if (it_control_type->second.SoTcontrol == dgsot::POSITION) {
      if (u_index != -1 && (urdf_joints[u_index]->limits)) {
        double lowerLim = urdf_joints[u_index]->limits->lower;
        ldiff = (aControl[lctl_index] - lowerLim);
        diff += ldiff;
        std::cout << "Position lowerLim: " << lowerLim << "\n"
                  << "motorcontrolSOUT: " << aControl[lctl_index]  << " -- "
                  << "diff: " << ldiff << "\n"
                  << "Velocity limit: " << urdf_joints[u_index]->limits->velocity
                  << std::endl;
      }
    } else {
      std::cout << "motorcontrolSOUT: " << aControl[lctl_index] << std::endl;
    }
  }
  std::cout << "\n ########### \n " << std::endl;
  std::cout << "totalDiff: " << diff << std::endl;

  BOOST_CHECK_CLOSE(diff, 0.0, 1e-3);
}
