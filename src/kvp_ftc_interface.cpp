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

#include <kuka_kvp_hw_interface/kvp_ftc_interface.h>

namespace kuka_kvp_hw_interface
{
KVPFTCInterface::KVPFTCInterface(ros::NodeHandle& nh) : nh_(nh)
{
  // Use default variables on the KRC if not given by parameter server
  nh.param("kvp_ftc", kvp_read_from_, std::string("FTC"));

  // Need IP on paramter server
  if (!nh.getParam("robot_ip_address", ip_))
  {
    ROS_ERROR("Cannot find required parameter 'robot_ip_address' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'robot_ip_address' on the parameter server.");
  }

  // Topic to publish geometry_msgs/WrenchStamped messages on
  nh_.param(std::string("force_torque_sensor_topic"), force_torque_sensor_topic_, std::string("ft_sensor/raw"));
  if (nh_.hasParam("force_torque_sensor_topic"))
  {
    ROS_WARN("Cannot find parameter 'force_torque_sensor_topic' on the parameter server, using default "
             "'ft_sensor/raw'");
  }

  // URDF frame asociated with sensor
  if (!nh_.getParam("force_torque_sensor_frame", force_torque_sensor_frame_))
  {
    ROS_ERROR("Cannot find required parameter 'force_torque_sensor_frame' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'force_torque_sensor_frame' on the parameter server.");
  }

  force_torque_sensor_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      force_torque_sensor_topic_, force_torque_sensor_frame_, force_, torque_));

  registerInterface(&force_torque_sensor_interface_);

  connect();
}

void KVPFTCInterface::read()
{
  std::map<std::string, std::string> ftc_data;
  // Read data from robot
  if (!robot_read_.readStruct(&kvp_read_from_, ftc_data))
  {
    ROS_ERROR("Failed to read from robot");
    return;
  }
  // @TODO: check input before parsing
  force_[0] = std::stod(ftc_data["Fx"]);
  force_[1] = std::stod(ftc_data["Fy"]);
  force_[2] = std::stod(ftc_data["Fz"]);
  torque_[0] = std::stod(ftc_data["Mx"]);
  torque_[0] = std::stod(ftc_data["My"]);
  torque_[0] = std::stod(ftc_data["Mz"]);
}

void KVPFTCInterface::connect()
{
  robot_read_.connectSocket(ip_, "7000");
}

void KVPFTCInterface::disconnect()
{
  robot_read_.disconnectSocket();
}

}  // namespace
