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

#include <kuka_kvp_hw_interface/kvp_joint_command_interface.h>

namespace kuka_kvp_hw_interface
{
KVPJointCommandInterface::KVPJointCommandInterface(ros::NodeHandle& nh)
  : nh_(nh), robot_update_start_(3), robot_update_done_(3)
{
  // Find joint names and set num_joints_
  industrial_utils::param::getJointNames(std::string("controller_joint_names"), std::string("robot_description"),
                                         joint_names_);
  num_joints_ = joint_names_.size();

  // Use default variables on the KRC if not given by parameter server
  nh.param("kvp_read_from", kvp_read_from_, std::string("$AXIS_ACT"));
  nh.param("kvp_write_to", kvp_write_to_, std::string("AXIS_SET"));

  // Need IP on paramter server
  if (!nh.getParam("robot_ip_address", ip_))
  {
    ROS_ERROR("Cannot find required parameter 'robot_ip_address' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'robot_ip_address' on the parameter server.");
  }

  // Resize joint states
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Resize joint commands
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "kuka_kvp_debug", 4));
  connect();
}

void KVPJointCommandInterface::read()
{
  // Release worker threads
  robot_update_start_.wait();

  // Wait for update to complete before returning
  robot_update_done_.wait();
}

void KVPJointCommandInterface::connect()
{
  robot_read_.connectSocket(ip_, "7000");
  robot_write_.connectSocket(ip_, "7000");
  read_thread_ = boost::thread(&KVPJointCommandInterface::readKVP, this);
  write_thread_ = boost::thread(&KVPJointCommandInterface::writeKVP, this);
}

void KVPJointCommandInterface::disconnect()
{
  robot_run_ = false;
  // Avoid deadlock, probably smother way to fix this
  robot_update_start_.wait();
  read_thread_.join();
  write_thread_.join();
  robot_read_.disconnectSocket();
  robot_write_.disconnectSocket();
}

void KVPJointCommandInterface::readKVP()
{
  double joint_position_rad[12];
  static const double DEG2RAD = 0.017453292519943295;

  while (true)
  {
    // Synchronise robot update threads
    robot_update_start_.wait();

    // Disconnect called
    if (!robot_run_)
      return;

    // Read data from robot
    if (!robot_read_.readE6AXIS(&kvp_read_from_, joint_position_rad))
    {
      ROS_ERROR("Failed to read from robot");
      robot_run_ = false;
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

    // Signal update done
    robot_update_done_.wait();
  }
}

void KVPJointCommandInterface::writeKVP()
{
  double joint_command_deg[num_joints_] = { 0 };
  static const double RAD2DEG = 57.295779513082323;

  while (true)
  {
    // Synchronise robot update threads
    robot_update_start_.wait();

    // Disconnect called
    if (!robot_run_)
      return;

    std::string debug;

    // Assume all robots have atleast 6 axes.
    // @TODO: Use one loop, get joint type from URDF
    for (int i = 0; i < 6; i++)
    {
      joint_command_deg[i] = RAD2DEG * joint_position_command_[i];
    }

    // Asume all external axes are linear
    // @TODO: Get axis type from URDF
    for (int i = 6; i < num_joints_; i++)
    {
      joint_command_deg[i] = 1000 * joint_position_command_[i];
    }

    // Write data to robot
    if (!robot_write_.writeE6AXIS(&kvp_write_to_, joint_command_deg, num_joints_, &debug))
    {
      ROS_ERROR("Failed to write to robot");
      robot_run_ = false;
      return;
    }

    // Add command position to debug info
    debug += "\n Command:";
    for (int i = 0; i < num_joints_; ++i)
    {
      debug += " J" + std::to_string(i) + ": " + std::to_string(joint_command_deg[i]);
    }

    // Publish string sent to robot
    if (realtime_pub_->trylock())
    {
      realtime_pub_->msg_.data = debug;
      realtime_pub_->unlockAndPublish();
    }

    // Signal update done
    robot_update_done_.wait();
  }  // while
}

}  // namespace
