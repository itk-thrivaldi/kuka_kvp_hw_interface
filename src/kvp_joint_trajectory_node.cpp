#include <ros/ros.h>
#include <kuka_kvp_hw_interface/kvp_joint_trajectory_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FollowJointTrajectoryAction");
  ros::NodeHandle nh;

  kuka_kvp_hw_interface::KVPJointTrajectoryInterface trajectory_interface(nh);
  ros::spin();

  return 0;
}
