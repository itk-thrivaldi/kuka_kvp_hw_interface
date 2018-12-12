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

#include <kuka_kvp_hw_interface/kvp_joint_command_loop.h>

namespace kuka_kvp_hw_interface
{
KVPJointCommandLoop::KVPJointCommandLoop(ros::NodeHandle& nh,
                                         boost::shared_ptr<kuka_kvp_hw_interface::KVPJointCommandInterface> robot)
  : nh_(nh), robot_(robot)
{
  cm_.reset(new controller_manager::ControllerManager(robot_.get(), nh_));

  // Use current time for first loop
  stopwatch_last_ = std::chrono::steady_clock::now();

  // Timer with callback
  control_loop_ = nh_.createTimer(ros::Duration(0.012), &KVPJointCommandLoop::controlUpdate, this);
}

void KVPJointCommandLoop::controlUpdate(const ros::TimerEvent& event)
{
  robot_->read();
  timestamp_ = ros::Time::now();
  stopwatch_now_ = std::chrono::steady_clock::now();
  period_.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now_ - stopwatch_last_).count());
  stopwatch_last_ = stopwatch_now_;
  cm_->update(timestamp_, period_);
}

}  // namespace
