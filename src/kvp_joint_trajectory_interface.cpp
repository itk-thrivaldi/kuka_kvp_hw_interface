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

#include <kuka_kvp_hw_interface/kvp_joint_trajectory_interface.h>

namespace kuka_kvp_hw_interface
{
KVPJointTrajectoryInterface::KVPJointTrajectoryInterface(ros::NodeHandle& nh)
  : nh_(nh), trajectory_barrier_(2), abort_barrier_(2)
{
  is_running_ = false;
  abort_trajectory_ = false;
  // Find joint names and set num_joints_
  industrial_utils::param::getJointNames(std::string("controller_joint_names"), std::string("robot_description"),
                                         joint_names_);
  num_joints_ = joint_names_.size();

  // Use default variables on the KRC if not given by parameter server
  nh_.param("kvp_write_to", kvp_write_to_, std::string("AXIS_SET"));

  // Need IP on paramter server
  if (!nh_.getParam("robot_ip_address", ip_))
  {
    ROS_ERROR("Cannot find required parameter 'ip' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'ip' on the parameter server.");
  }

  // Start trajectory execution thread
  trajectory_thread_ = boost::thread(&KVPJointTrajectoryInterface::executeTrajectory, this);

  // Connect to robot
  connect();

  action_server_.reset(new ActionServer(nh_, "joint_trajectory_action",
                                        boost::bind(&KVPJointTrajectoryInterface::goalCB, this, _1),
                                        boost::bind(&KVPJointTrajectoryInterface::cancelCB, this, _1), false));
  action_server_->start();
}

void KVPJointTrajectoryInterface::cancelCB(ActionServer::GoalHandle gh)
{
  ROS_DEBUG("Actionlib cancel callback");

  // Make sure the cancel CB is for the current GoalHandle
  mutex_.lock();
  bool abort = gh_ == gh;
  mutex_.unlock();

  if (is_running_.load() && abort)
  {
    abort_trajectory_ = true;
    abort_barrier_.wait();
  }
}

void KVPJointTrajectoryInterface::goalCB(ActionServer::GoalHandle gh)
{
  // Check that joint names match
  if (!std::is_permutation(gh.getGoal()->trajectory.joint_names.begin(), gh.getGoal()->trajectory.joint_names.end(),
                           joint_names_.begin()))
  {
    ROS_ERROR("Joints on incoming goal don't match the controller joints.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  gh.setAccepted();

  // Try to update new trajectory
  // Check for, and abort, any in-progress executions
  if (is_running_.load())
  {
    // Abort
    abort_trajectory_ = true;
    // Wait for abort OK
    abort_barrier_.wait();
  }

  // Keep copy of GoalHandle
  mutex_.lock();
  gh_ = gh;
  mutex_.unlock();

  // Signal trajectory execution start
  trajectory_barrier_.wait();
}

void KVPJointTrajectoryInterface::executeTrajectory()
{
  // $POS_BACK Start position of current motion block. Cartesian
  // $AXIS_BACK Start position of current motion block E6AXIS
  // $AXIS_FOR Target position of current motion block E6AXIS

  while (ros::ok())
  {
    // Wait for trajectory
    trajectory_barrier_.wait();

    // Lock mutex
    mutex_.lock();
    // Copy trajectory
    auto trajectory = gh_.getGoal()->trajectory;
    // Update state to running
    is_running_ = true;
    // Unlock mutex
    mutex_.unlock();

    ROS_INFO("Got new trajectory with %lu points", trajectory.points.size());
    // Loop over points
    for (auto point : trajectory.points)
    {
      // Map point to E6AXIS
      std::map<std::string, double> joint_command = trajectoryPoint2e6axis(trajectory.joint_names, point);

      // Send point to robot
      robot_.writeE6AXIS(&kvp_write_to_, joint_command, num_joints_);

      // Wait for execution of point to start
      while (!isCurrentMotion(joint_command))
      {
        // Abort if abort
        if (abort_trajectory_.load())
        {
          break;
        }

        // Don't spam controller
        //    ros::Duration(0.1).sleep();
      }

      // Should we abort?
      if (abort_trajectory_.load())
      {
        mutex_.lock();
        // Clear aborted flag
        abort_trajectory_ = false;
        // Set state not running
        is_running_ = false;
        // Cancel current goal
        gh_.setCanceled();
        // unlock mutex
        mutex_.unlock();
        // Signal abort OK
        abort_barrier_.wait();
        // break
        break;
      }
    }
    // Execution complete
    if (is_running_.load())
    {
      mutex_.lock();
      gh_.setSucceeded();
      is_running_ = false;
      mutex_.unlock();
    }
  }  // while
}

std::map<std::string, double> KVPJointTrajectoryInterface::trajectoryPoint2e6axis(
    std::vector<std::string> traj_joint_names, trajectory_msgs::JointTrajectoryPoint point)
{
  static const double RAD2DEG = 57.295779513082323;
  std::map<std::string, double> tmp;
  std::map<std::string, double> e6axis;
  std::string kuka_joints[12] = { "A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6" };

  // Assumptions:
  // joint_names_ from parameter server is in order A1-A6 E1-E6
  // A1-A6 is revolute
  // E1-E6 is linear

  // Order joints from A1 to E6.
  // Done in two steps we don't controll the trajectory order

  // Move points to a named array
  for (size_t i = 0; i < num_joints_; i++)
  {
    tmp[traj_joint_names[i]] = point.positions[i];
  }

  // Create E6AXIS map
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    e6axis[kuka_joints[i]] = tmp[joint_names_[i]];
  }

  // End order joints

  // Convert A1-A6 to degrees
  for (size_t i = 0; i < 6; i++)
  {
    e6axis[kuka_joints[i]] = e6axis[kuka_joints[i]] * RAD2DEG;
  }

  // Convert E1-E6 to mm
  for (size_t i = 6; i < num_joints_; i++)
  {
    e6axis[kuka_joints[i]] = e6axis[kuka_joints[i]] * 1000;
  }

  return e6axis;
}

bool KVPJointTrajectoryInterface::isCurrentMotion(std::map<std::string, double> wanted)
{
  std::map<std::string, double> actual;
  std::string read_from = "$AXIS_FOR";

  if (!robot_.readE6AXIS(&read_from, actual))
  {
    ROS_DEBUG("Error reading from robot. Is $AXIS_FOR empty?");

    // Check that we are still connected to robot
    if (!robot_.checkSocket())
    {
      ROS_ERROR("Disconnected from robot");
      throw std::runtime_error("Disconnected from robot");
    }
  }

  // Do a rough compare
  for (auto& it : actual)
  {
    if ((it.second > wanted[it.first] + 2) || (it.second < wanted[it.first] - 2))
    {
      return false;
    }
  }

  return true;
}

void KVPJointTrajectoryInterface::connect()
{
  robot_.connectSocket(ip_, "7000");
}

void KVPJointTrajectoryInterface::disconnect()
{
  robot_.disconnectSocket();
}

}  // namespace
