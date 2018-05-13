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
*/

#ifndef KVP_JOINT_TRAJECTORY_INTERFACE_H
#define KVP_JOINT_TRAJECTORY_INTERFACE_H

// C++
#include <string>
#include <vector>
#include <algorithm>

// Boost
#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>

// ROS
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ROS industrial
#include <industrial_utils/param_utils.h>

// KVP communication
#include <BoostClientCross.h>

namespace kuka_kvp_hw_interface
{
class KVPJointTrajectoryInterface
{
protected:
  ros::NodeHandle nh_;
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;
  boost::shared_ptr<ActionServer> action_server_;

  // Configuration variables
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;

  // Variables relating to KVP configuration
  std::string ip_;
  std::string kvp_write_to_;

  bool isCurrentMotion(std::map<std::string, double> wanted);
  std::map<std::string, double> trajectoryPoint2e6axis(std::vector<std::string> joint_names,
                                                       trajectory_msgs::JointTrajectoryPoint point);

private:
  // Variables for robot communication
  BoostClientCross robot_;
  boost::mutex mutex_;
  boost::barrier trajectory_barrier_;
  boost::barrier abort_barrier_;
  boost::thread trajectory_thread_;

  std::atomic<bool> is_running_;
  std::atomic<bool> abort_trajectory_;

  ActionServer::GoalHandle gh_;

  void executeTrajectory();

public:
  KVPJointTrajectoryInterface(ros::NodeHandle& nh);

  /** \brief Destructor. Calls disconnect */
  ~KVPJointTrajectoryInterface()
  {
    disconnect();
  };

  void cancelCB(ActionServer::GoalHandle gh);
  void goalCB(ActionServer::GoalHandle gh);

  /** \brief Disconnect from robot */
  void disconnect();

  /**
   * @brief Connect to robot
   */
  void connect();
};  // class

}  // namespace

#endif
