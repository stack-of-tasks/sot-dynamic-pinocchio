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
#include <yaml-cpp/yaml.h>
#include <pinocchio/parsers/urdf.hpp>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot-dynamic-pinocchio/pinocchio-device.hh>

#include <sstream>
#include <fstream>

void CreateYAMLFILE()
{
  YAML::Emitter yaml_out;
  YAML::Node aNode,yn_map_hw_sot_cm,yn_map_sensors;
  unsigned int index_vec_ctl=0;
  yn_map_hw_sot_c = aNode["map_hardware_sot_control"];
  yn_map_sensors = aNode["sensors"];
  yn_map_hw_sot_c["waist"];
  yn_map_hw_sot_c["waist"]["hw"] = "POSITION";
  yn_map_hw_sot_c["waist"]["sot"] = "POSITION";
  yn_map_hw_sot_c["waist"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["waist"]["sensors"] = "";
  index_vec_ctl+=6;
  
  yn_map_hw_sot_c["RLEG_HIP_P"];
  yn_map_hw_sot_c["RLEG_HIP_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RLEG_HIP_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RLEG_HIP_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["RLEG_HIP_R"];
  yn_map_hw_sot_c["RLEG_HIP_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RLEG_HIP_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RLEG_HIP_Y"];
  yn_map_hw_sot_c["RLEG_HIP_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RLEG_HIP_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RLEG_HIP_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;


  yn_map_hw_sot_c["RLEG_KNEE"];
  yn_map_hw_sot_c["RLEG_KNEE"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_KNEE"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RLEG_KNEE"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RLEG_KNEE"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RLEG_ANKLE_P"];
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["RLEG_ANKLE_R"];
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["sot"] = "VELOCITY";  
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LLEG_HIP_P"];
  yn_map_hw_sot_c["LLEG_HIP_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_HIP_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_HIP_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  
  yn_map_hw_sot_c["LLEG_HIP_R"];
  yn_map_hw_sot_c["LLEG_HIP_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_HIP_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_HIP_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["LLEG_HIP_Y"];
  yn_map_hw_sot_c["LLEG_HIP_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_HIP_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_HIP_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";  
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["LLEG_KNEE"];
  yn_map_hw_sot_c["LLEG_KNEE"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_KNEE"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_KNEE"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_KNEE"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";  
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["LLEG_ANKLE_P"];
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LLEG_ANKLE_R"];
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RARM_SHOULDER_P"];
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RARM_SHOULDER_R"];
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]"; 
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RARM_SHOULDER_Y"];
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RARM_ELBOW"];
  yn_map_hw_sot_c["RARM_ELBOW"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_ELBOW"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_ELBOW"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_ELBOW"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["RARM_WRIST_Y"];
  yn_map_hw_sot_c["RARM_WRIST_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_WRIST_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_WRIST_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["RARM_WRIST_P"];
  yn_map_hw_sot_c["RARM_WRIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_WRIST_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_WRIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["RARM_WRIST_R"];
  yn_map_hw_sot_c["RARM_WRIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["RARM_WRIST_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["RARM_WRIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current]";   
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LARM_SHOULDER_P"];
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LARM_SHOULDER_R"];
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LARM_SHOULDER_Y"];
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  
  yn_map_hw_sot_c["LARM_ELBOW"];
  yn_map_hw_sot_c["LARM_ELBOW"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_ELBOW"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_ELBOW"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_ELBOW"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";  
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LARM_WRIST_Y"];
  yn_map_hw_sot_c["LARM_WRIST_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_Y"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_WRIST_Y"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_WRIST_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["LARM_WRIST_P"];
  yn_map_hw_sot_c["LARM_WRIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_WRIST_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_WRIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";  
  index_vec_ctl+=1;  

  yn_map_hw_sot_c["LARM_WRIST_R"];
  yn_map_hw_sot_c["LARM_WRIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["LARM_WRIST_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["LARM_WRIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current]";
  index_vec_ctl+=1;

  yn_map_hw_sot_c["WAIST_P"];
  yn_map_hw_sot_c["WAIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["WAIST_P"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["WAIST_P"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["WAIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;  

  yn_map_hw_sot_c["WAIST_R"];
  yn_map_hw_sot_c["WAIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["WAIST_R"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["WAIST_R"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["WAIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;
  
  yn_map_hw_sot_c["CHEST"];
  yn_map_hw_sot_c["CHEST"]["hw"] = "POSITION";
  yn_map_hw_sot_c["CHEST"]["sot"] = "VELOCITY";
  yn_map_hw_sot_c["CHEST"]["controlPos"] = index_vec_ctl;
  yn_map_hw_sot_c["CHEST"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";
  index_vec_ctl+=1;

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["left_ankle_ft"];
  yn_map_sensors["force_torque"]["left_ankle_ft"]["sensor_joint"] = "LLEG_ANKLE_R";
  yn_map_sensors["force_torque"]["left_ankle_ft"]["frame"] = "LLEG_LINK_6";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["right_ankle_ft"];
  yn_map_sensors["force_torque"]["right_ankle_ft"]["sensor_joint"] = "RLEG_ANKLE_R";
  yn_map_sensors["force_torque"]["right_ankle_ft"]["frame"] = "RLEG_LINK_6";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["left_wrist_ft"];
  yn_map_sensors["force_torque"]["left_wrist_ft"]["sensor_joint"] = "LLEG_WRIST_R";
  yn_map_sensors["force_torque"]["left_wrist_ft"]["frame"] = "LARM_LINK_7";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["right_wrist_ft"];
  yn_map_sensors["force_torque"]["right_wrist_ft"]["sensor_joint"] = "RLEG_WRIST_R";
  yn_map_sensors["force_torque"]["right_wrist_ft"]["frame"] = "RARM_LINK_7";


  YAML::Node yn_imu = yn_map_sensors["imu"];
  yn_imu["base_imu"];
  yn_imu["base_imu"]["frame"]="LARM_LINK6";
  yn_imu["base_imu"]["gazebo_sensor_J"]="LARM_LINK6";

  ofstream of;
  of.open("map_hs_sot_gen.yaml",ios::out);
  if (of.is_open())
    {
      of << aNode;
    }
  of.close();
}

int ReadYAMLFILE(dg::sot::PinocchioDevice &aDevice, unsigned int debug_mode)
{
  std::string yaml_file = "map_hs_sot_gen.yaml";
  YAML::Node map_hs_sot = YAML::LoadFile(yaml_file);
  
  if (map_hs_sot.IsNull())
    {
      std::cerr << "Unable to read " << yaml_file << std::endl;
      return -1;
    }
  if (debug_mode>0)
    std::cout << "Reading file : "<< yaml_file << std::endl;
  YAML::Node map_hs_control = map_hs_sot["map_hardware_sot_control"];
  if (debug_mode>1)
    {
      std::cout << "map_hs_control.size(): "
		<< map_hs_control.size() << std::endl;
      std::cout << map_hs_control << std::endl;
    }
  unsigned int i=0;
  for (YAML::const_iterator it=map_hs_control.begin();
       it!=map_hs_control.end();
       it++)
    {
      if (debug_mode>1)
	{
	  std::cout << i << " " << std::endl;
	  std::cout << "key:" << it->first.as<string>() << std::endl;
	}
      std::string jointName = it->first.as<string>();
      
      YAML::Node aNode = it->second;
      if (debug_mode>1)
	std::cout << "Type of value: " << aNode.Type() << std::endl;
      
      for (YAML::const_iterator it2=aNode.begin();
	   it2!=aNode.end();
	   it2++)
	
	{
	  std::string aKey = it2->first.as<string>();
	  if (debug_mode>1)
	    std::cout << "-- key:" << aKey << std::endl;

	  if (aKey=="hw")
	    {
	      std::string value = it2->second.as<string>();
	      if (debug_mode>1)
		std::cout << "-- Value: " << value << std::endl;
	      aDevice.setHWControlType(jointName,value);
	    }
	  else if (aKey=="sot")
	    {
	      std::string value = it2->second.as<string>();
	      if (debug_mode>1)
		std::cout << "-- Value: " << value << std::endl;
	      aDevice.setSoTControlType(jointName,value);
	    }
	  else if (aKey=="controlPos")
	    {
	      unsigned int index= it2->second.as<int>();
	      if (debug_mode>1)
		std::cout << "-- index: " << index << std::endl;
	      aDevice.setControlPos(jointName,index);
	    }
	}
      i++;
    }
  return 0;
}

namespace dg = dynamicgraph;
int main(int, char **)
{
  unsigned int debug_mode=0;
    
  std::string robot_description;
  ifstream urdfFile;
  std::string filename="/opt/openrobots/share/simple_humanoid_description/urdf/simple_humanoid.urdf";
  urdfFile.open(filename.c_str());
  if (!urdfFile.is_open())
    {
      std::cerr << "Unable to open " << filename << std::endl;
      return -1;
    }
  stringstream strStream;
  strStream << urdfFile.rdbuf();
  robot_description = strStream.str();
  
  /// Test reading the URDF file.
  dg::sot::PinocchioDevice aDevice(std::string("simple_humanoid"));

  aDevice.setURDFModel(robot_description);
  CreateYAMLFILE();

  if(ReadYAMLFILE(aDevice, debug_mode)<0)
    return -1;

  dg::Vector aState(29);
  for(unsigned j=0;j<aState.size();j++)
    aState(j)=0.0;
  aDevice.setState(aState);
  
  /// Fix constant vector for the control entry
  dg::Vector aControlVector(35);
  double dt=0.005;
  for(unsigned int i=0;i<35;i++)
    aControlVector[i] = -0.5;
  aDevice.controlSIN.setConstant(aControlVector);

  for(unsigned int i=0;i<2000;i++)
    aDevice.increment(dt);

  const se3::Model & aModel = aDevice.getModel();

  const dg::Vector & aPosition = aDevice.stateSOUT(2001);
  double diff=0;
  for (unsigned int j=0;j<aPosition.size();j++)
    {
      diff += fabs(aPosition(j) - aModel.lowerPositionLimit[j]);
      if (debug_mode>1)
	std::cout << diff << " " << aModel.names[j] << " " 
		  << aPosition(j) << " "<<  aModel.lowerPositionLimit[j]
		  << std::endl;
    }
  if (diff>1e-3)
    return -1;
  return 1;
}
