# kuka_kvp_hw_interface

Package for controlling a KUKA KRC robot from ROS using KUKAVARPROXY


## ROS Nodes in this package

* kuka_kvp_node

   Publishes robot joint states using ROS Controls JointStateHandle
   Sets robot joints using JointPositionCommand. Works good with JointGroupPosition controller, but might skip trajectory points if used with FollowJointTrajectory controller

* kvp_ftc_node

   For publishing data from a force torque sensor. Sensordata must be stored to a struc on the KRC.
   Config via parameter server:
   * force_torque_sensor_topic Where to publish WrenchStamped messages. Defaults to ft_sensor/raw
   * force_torque_sensor_frame What frame is the FT-sensor connected to.

* kvp_sws_node

  Service node for controlling toolchanger
  * /get_tool_status Get current status of tool.
  * /lock_tool Locks toolchanger
  * /unlock_tool Unlocks toolchanger

