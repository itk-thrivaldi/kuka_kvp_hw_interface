/**
Copyright 2017 Ivar Eriksen

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.
*/

/* Author: Ivar Eriksen
   Desc: Control toolchanger on a KUKA robot using KUKAVARPROXY (KVP).
*/

#include <kuka_kvp_hw_interface/kvp_sws_interface.h>

namespace kuka_kvp_hw_interface
{
KVPSWSInterface::KVPSWSInterface(ros::NodeHandle& nh) : nh_(nh)
{
  // @TODO: Get these from parameter server?
  sws_status = "sws_status";
  sws_set = "sws_set";

  // Need IP on paramter server
  if (!nh_.getParam("robot_ip_address", ip_))
  {
    ROS_ERROR("Cannot find required parameter 'robot_ip_address' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'robot_ip_address' on the parameter server.");
  }
}

bool KVPSWSInterface::getToolStatus(kuka_kvp_hw_interface::getToolStatus::Request& req,
                                    kuka_kvp_hw_interface::getToolStatus::Response& res)
{
  BoostClientCross robot;
  robot.connectSocket(ip_, "7000");
  bool status;

  if (!robot.readBool(&sws_status, status))
  {
    ROS_ERROR("Failed to read from robot");
    res.status.val = industrial_msgs::TriState::UNKNOWN;
    robot.disconnectSocket();
    return true;
  }

  if (status)
  {
    res.status.val = industrial_msgs::TriState::ON;
  }
  else
  {
    res.status.val = industrial_msgs::TriState::OFF;
  }
  robot.disconnectSocket();
  return true;
}

bool KVPSWSInterface::lock(kuka_kvp_hw_interface::lockTool::Request& req,
                           kuka_kvp_hw_interface::lockTool::Response& res)
{
  if (write(true))
  {
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
  }

  return true;
}

bool KVPSWSInterface::unlock(kuka_kvp_hw_interface::lockTool::Request& req,
                             kuka_kvp_hw_interface::lockTool::Response& res)
{
  if (write(false))
  {
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
  }

  return true;
}

bool KVPSWSInterface::write(bool lock)
{
  bool status;
  BoostClientCross robot;
  robot.connectSocket(ip_, "7000");

  // Write value
  if (!robot.writeBool(&sws_set, &lock))
  {
    ROS_ERROR("Failed to write to robot");
    robot.disconnectSocket();
    return false;
  }

  // Wait for script to run on robot
  // Hackish way to do this
  ros::Duration(0.1).sleep();

  // Poll value from robot to check success
  if (!robot.readBool(&sws_status, status))
  {
    ROS_ERROR("Failed to read from robot");
    robot.disconnectSocket();
    return false;
  }

  robot.disconnectSocket();

  if (status == lock)
  {
    return true;
  }

  return false;
}
}  // namespace
