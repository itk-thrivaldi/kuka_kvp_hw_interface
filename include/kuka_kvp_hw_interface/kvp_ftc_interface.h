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

#ifndef KVP_FTC_INTERACE_H
#define KVP_FTC_INTERFACE_H

// C++
#include <string>
#include <vector>

// ROS
#include <std_msgs/String.h>
#include <ros/ros.h>

// ROS control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>

// KVP communication
#include <BoostClientCross.h>

namespace kuka_kvp_hw_interface
{
/**
 * @brief Read data from a force-torque sensor and publish
 */
class KVPFTCInterface : public hardware_interface::RobotHW
{
protected:
  ros::NodeHandle nh_;

  // Configuration variables
  std::string force_torque_sensor_topic_;
  std::string force_torque_sensor_frame_;

  // For use with ros_control force_torque_sensor
  double force_[3];
  double torque_[3];
  hardware_interface::ForceTorqueSensorInterface force_torque_sensor_interface_;

  // Variables relating to KVP configuration
  std::string ip_;
  std::string kvp_read_from_;

private:
  // Variables for robot communication
  BoostClientCross robot_read_;

public:
  KVPFTCInterface(ros::NodeHandle& nh);

  /** \brief Read state from robot
   *   Releases both read and write threads
   */
  void read();

  /** \brief Connects to robot and starts worker threads */
  void connect();

  /** \brief Write to robot
   *Both read and write threads are released from read() function
   */
  void write(){};

  /** \brief Destructor. Calls disconnect */
  ~KVPFTCInterface()
  {
    disconnect();
  };

  /** \brief Stop threads and disconnect from robot */
  void disconnect();

};  // class

}  // namespace

#endif
