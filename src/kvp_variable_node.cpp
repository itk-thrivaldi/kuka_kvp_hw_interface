#include <ros/ros.h>

#include <kuka_kvp_hw_interface/kvp_variable_interface.h>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "kvp_variable_interface");
  ros::NodeHandle nh;

  kuka_kvp_hw_interface::KVPVariableInterface interface(nh);

  ros::ServiceServer getBool =
    nh.advertiseService("get_bool", &kuka_kvp_hw_interface::KVPVariableInterface::getBool, &interface);
  ros::ServiceServer setBool =
    nh.advertiseService("set_bool", &kuka_kvp_hw_interface::KVPVariableInterface::setBool, &interface);
  ROS_INFO("KVP Variable node ready");
  ros::spin();

  return 0;
}
