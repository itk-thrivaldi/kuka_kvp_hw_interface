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
#include <kuka_kvp_hw_interface/kvp_joint_state_loop.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuka_kvp_hardware_interface");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  boost::shared_ptr<kuka_kvp_hw_interface::KVPJointStateInterface> robot(
      new kuka_kvp_hw_interface::KVPJointStateInterface(nh));
  ROS_INFO_STREAM_NAMED("hardware_interface", "Connected to robot");

  ROS_DEBUG_STREAM_NAMED("hardware_interface", "Entering loop");
  kuka_kvp_hw_interface::KVPJointStateLoop control_loop(nh, robot);

  while (ros::ok())
  {
  };

  robot->disconnect();
  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;
}
