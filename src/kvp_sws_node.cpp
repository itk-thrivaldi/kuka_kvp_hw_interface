#include <ros/ros.h>

#include <kuka_kvp_hw_interface/kvp_sws_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kvp_sws_interface");
  ros::NodeHandle nh;

  kuka_kvp_hw_interface::KVPSWSInterface interface(nh);

  ros::ServiceServer status =
      nh.advertiseService("get_tool_status", &kuka_kvp_hw_interface::KVPSWSInterface::getToolStatus, &interface);
  ros::ServiceServer unlockTool =
      nh.advertiseService("unlock_tool", &kuka_kvp_hw_interface::KVPSWSInterface::unlock, &interface);
  ros::ServiceServer lockTool =
      nh.advertiseService("lock_tool", &kuka_kvp_hw_interface::KVPSWSInterface::lock, &interface);

  ROS_INFO("SWS Toolchanger node ready");
  ros::spin();

  return 0;
}
