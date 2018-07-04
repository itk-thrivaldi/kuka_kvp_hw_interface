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

#ifndef KVP_SWS_INTERACE_H
#define KVP_SWS_INTERFACE_H

// C++
#include <string>
#include <vector>

// ROS
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <industrial_msgs/TriState.h>
#include <industrial_msgs/ServiceReturnCode.h>
#include <kuka_kvp_hw_interface/lockTool.h>
#include <kuka_kvp_hw_interface/unlockTool.h>
#include <kuka_kvp_hw_interface/getToolStatus.h>

// KVP communication
#include <BoostClientCross.h>

namespace kuka_kvp_hw_interface
{
/**
 * @brief Control toolchanger via KUKAVARPROXY
 */
class KVPSWSInterface
{
protected:
  ros::NodeHandle nh_;

  // Variables relating to KVP configuration
  std::string ip_;
  std::string sws_status;
  std::string sws_set;

  /**
   * @brief Private method for sending variable to robot
   *
   * @param lock True to lock toolchanger, false to unlock
   *
   * @return True on success
   */
  bool write(bool lock);

public:
  KVPSWSInterface(ros::NodeHandle& nh);

  /** \brief Get state of toolchanger
   */
  bool getToolStatus(kuka_kvp_hw_interface::getToolStatus::Request& req,
                     kuka_kvp_hw_interface::getToolStatus::Response& res);

  /** \brief Lock toolchanger
   */
  bool lock(kuka_kvp_hw_interface::lockTool::Request& req, kuka_kvp_hw_interface::lockTool::Response& res);

  /** \brief Unlock toolchanger
   */
  bool unlock(kuka_kvp_hw_interface::lockTool::Request& req, kuka_kvp_hw_interface::lockTool::Response& res);

};  // class

}  // namespace

#endif
