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

#ifndef KVP_JOINT_STATE_LOOP_H
#define KVP_JOINT_STATE_LOOP_H

#include <kuka_kvp_hw_interface/kvp_joint_state_interface.h>
#include <chrono>
#include <ros/ros.h>

namespace kuka_kvp_hw_interface
{
/**
 * @brief HW Controll loop
 */
class KVPJointStateLoop
{
public:
  KVPJointStateLoop(ros::NodeHandle& nh, boost::shared_ptr<kuka_kvp_hw_interface::KVPJointStateInterface> robot);
  void controlUpdate(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_;
  ros::Timer control_loop_;
  boost::shared_ptr<controller_manager::ControllerManager> cm_;
  boost::shared_ptr<kuka_kvp_hw_interface::KVPJointStateInterface> robot_;
  std::chrono::time_point<std::chrono::steady_clock> stopwatch_last_;
  std::chrono::time_point<std::chrono::steady_clock> stopwatch_now_;
  ros::Time timestamp_;
  ros::Duration period_;

};  // class

}  // namespace

#endif
