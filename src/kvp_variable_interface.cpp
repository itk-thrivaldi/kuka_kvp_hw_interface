/**
Copyright 2018 Mathias Hauan Arbo

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.
*/

/* Author: Mathias Hauan Arbo
   Desc: Read and write from the global variables on the KRC using
   KUKAVARPROXY (KVP). Currently only supports Booleans, but can be
   expanded.
*/

#include <kuka_kvp_hw_interface/kvp_variable_interface.h>

namespace kuka_kvp_hw_interface {
  KVPVariableInterface::KVPVariableInterface(ros::NodeHandle& nh) : nh_(nh) {
    if (!nh_.getParam("robot_ip_address", ip_)) {
      ROS_ERROR("Cannot find required parameter 'robot_ip_address' "
		"on the parameter server.");
      throw std::runtime_error("Cannot find required parameter "
			       "'robot_ip_address' on the parameter server.");
    }
  }
  
  bool KVPVariableInterface::writeBool(const std::string& name, bool value) {
    bool status;
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000"); // TODO: Get port from param server
    // Write value
    if (!robot.writeBool(&name, &value)) {
      ROS_ERROR("Failed to write to robot.");
      robot.disconnectSocket();
      return false;
    }
    // Wait for SPS to run on robot
    // Horribly hackish way to do this...
    ros::Duration(0.1).sleep();

    // Poll value from robot to check success
    if (!robot.readBool(&name, status)) {
      ROS_ERROR("Failed to read from robot.");
      robot.disconnectSocket();
      return false;
    }
    robot.disconnectSocket();
    if (status == value) {
      return true;
    }
    return false;
  }

  bool KVPVariableInterface::writeInt(const std::string& name, int value) {
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000"); // TODO: get port from param server
    // Write value
    if (!robot.writeInt(&name, &value)) {
      ROS_ERROR("Failed to write to robot.");
      robot.disconnectSocket();
      return false; 
    }
    // TODO: check for success
    robot.disconnectSocket();
    return true;
  }

  bool KVPVariableInterface::writeFloat(const std::string& name, double value) {
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000");

    // Write value
    if (!robot.writeReal(&name, &value)) {
      ROS_ERROR("Failed to write to robot.");
      robot.disconnectSocket();
      return false;
    }
    // TODO: check for succes
    robot.disconnectSocket();
    return true;
  }
  
  bool KVPVariableInterface::getBool(kuka_kvp_hw_interface::GetBool::Request& req,
				     kuka_kvp_hw_interface::GetBool::Response& res) {
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000"); // TODO: Get port from param server
    bool status;

    if (!robot.readBool(&req.name, status)) {
      ROS_ERROR("Failed to read from robot");
      res.status.val = industrial_msgs::TriState::UNKNOWN;
      robot.disconnectSocket();
      return false;
    }

    if (status) {
      res.status.val = industrial_msgs::TriState::TRUE;
    } else {
      res.status.val = industrial_msgs::TriState::FALSE;
    }
    robot.disconnectSocket();
    return true;
  }

  bool KVPVariableInterface::setBool(kuka_kvp_hw_interface::SetBool::Request& req,
				     kuka_kvp_hw_interface::SetBool::Response& res) {
    if (writeBool(req.name, req.value)) {
      res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
    } else {
      res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
    }
    return true;
  }

  bool KVPVariableInterface::getInt(kuka_kvp_hw_interface::GetInt::Request& req,
				    kuka_kvp_hw_interface::GetInt::Response& res) {
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000"); //TODO: get port from parameter server
    int value;
    if (!robot.readInt(&req.name, value)) {
      ROS_ERROR("Failed to read from robot.");
      robot.disconnectSocket();
      return false;
    }

    res.value = value;
    robot.disconnectSocket();
    return true;
  }

  bool KVPVariableInterface::setInt(kuka_kvp_hw_interface::SetInt::Request& req,
				    kuka_kvp_hw_interface::SetInt::Response& res) {
    if (writeInt(req.name, req.value)) {
      res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
    } else {
      res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
    }
    return true;
  }

  bool KVPVariableInterface::getFloat(kuka_kvp_hw_interface::GetFloat::Request& req,
				      kuka_kvp_hw_interface::GetFloat::Response& res) {
    BoostClientCross robot;
    robot.connectSocket(ip_, "7000"); //TODO: get port from parameter server
    double value;
    if (!robot.readReal(&req.name, value)) {
      ROS_ERROR("Failed to read from robot.");
      robot.disconnectSocket();
      return false;
    }

    res.value = value;
    robot.disconnectSocket();
    return true;
  }

  bool KVPVariableInterface::setFloat(kuka_kvp_hw_interface::SetFloat::Request& req,
				      kuka_kvp_hw_interface::SetFloat::Response& res) {
    if (writeFloat(req.name, req.value)) {
      res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
    } else {
      res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
    }
    return true;
  }

}
