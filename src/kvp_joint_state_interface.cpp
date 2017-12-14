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
   Desc: Control a KUKA robot using KUKAVARPROXY (KVP).
         Inspired from davetcoleman/ros_control_boilerplate and kuka_experimental/kuka_rsi_hw_interface
*/

#include <kuka_kvp_hw_interface/kvp_joint_state_interface.h>

namespace kuka_kvp_hw_interface
{
KVPJointStateInterface::KVPJointStateInterface(ros::NodeHandle& nh) : nh_(nh)
{
  // Find joint names and set num_joints_
  industrial_utils::param::getJointNames(std::string("controller_joint_names"), std::string("robot_description"),
                                         joint_names_);
  num_joints_ = joint_names_.size();

  // Use default variables on the KRC if not given by parameter server
  nh.param("kvp_read_from", kvp_read_from_, std::string("$AXIS_ACT"));

  // Need IP on paramter server
  if (!nh.getParam("robot_ip_address", ip_))
  {
    ROS_ERROR("Cannot find required parameter 'ip' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'ip' on the parameter server.");
  }

  // Resize joint states
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));
  }
  registerInterface(&joint_state_interface_);

  connect();
}

void KVPJointStateInterface::read()
{
  double joint_position_rad[12];
  static const double DEG2RAD = 0.017453292519943295;

  // Read data from robot
  if (!robot_read_.readE6AXIS(&kvp_read_from_, joint_position_rad))
  {
    ROS_ERROR("Failed to read from robot");
    return;
  }

  // Assume all robots have atleast 6 axes.
  // @TODO: Use one loop, get joint type from URDF
  for (int i = 0; i < 6; i++)
  {
    joint_position_[i] = DEG2RAD * joint_position_rad[i];
  }

  // Assume all external axes are linear
  // @TODO: Get axis type from URDF
  for (int i = 6; i < num_joints_; i++)
  {
    joint_position_[i] = joint_position_rad[i] / 1000;
  }
}

void KVPJointStateInterface::connect()
{
  robot_read_.connectSocket(ip_, "7000");
}

void KVPJointStateInterface::disconnect()
{
  robot_read_.disconnectSocket();
}

}  // namespace
