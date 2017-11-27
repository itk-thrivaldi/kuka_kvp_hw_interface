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

#include <chrono>

#include <kuka_kvp_hw_interface/kvp_ftc_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kvp_ftc_hardware_interface");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  kuka_kvp_hw_interface::KVPFTCInterface robot(nh);
  controller_manager::ControllerManager cm(&robot);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Connected to robot");

  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  ros::Rate r(10);  // never use this for anything with remotely RT demands

  ROS_INFO_STREAM_NAMED("hardware_interface", "Entering loop");
  while (ros::ok())
  {
    robot.read();
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;
    cm.update(timestamp, period);
    r.sleep();
  }

  robot.disconnect();
  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;
}
