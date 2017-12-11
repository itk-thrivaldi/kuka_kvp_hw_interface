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

  action_server_.reset(new ActionServer(nh_, "follow_joint_trajectory",
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
  std::vector<std::string> vec1 = gh.getGoal()->trajectory.joint_names;
  std::vector<std::string> vec2 = joint_names_;
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

    ROS_INFO("Got new trajectory with %i points", trajectory.points.size());
    // Loop over points
    for (auto point : trajectory.points)
    {
			// Order points according to joint_names_
			double joint_command[num_joints_];
			traj2joint(trajectory.joint_names, point, joint_command);

      // Send point to robot
      robot_.writeE6AXIS(&kvp_write_to_, joint_command, num_joints_);

      // Map point to E6AXIS
      std::map<std::string, double> e6axis = trajectoryPoint2exa6is(trajectory.joint_names, point);
      // Wait for execution of point to start
      while (!isCurrentMotion(joint_command))
      {
        // Abort if abort
        if(abort_trajectory_.load()) {
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

void KVPJointTrajectoryInterface::traj2joint(std::vector<std::string> joint_names, trajectory_msgs::JointTrajectoryPoint point, double *joint_command) {
  static const double RAD2DEG = 57.295779513082323;
  std::map<std::string, double> tmp;
	for (size_t i = 0; i < joint_names.size(); ++i)
  {
    tmp[joint_names[i]] = point.positions[i] * RAD2DEG;
  }

  for (std::size_t i = 0; i < num_joints_; i++)
  {
    joint_command[i] = tmp[joint_names_[i]];
  }
}

std::map<std::string, double> KVPJointTrajectoryInterface::trajectoryPoint2exa6is(
    std::vector<std::string> joint_names, trajectory_msgs::JointTrajectoryPoint point)
{
  static const double RAD2DEG = 57.295779513082323;
  std::map<std::string, double> tmp;
  std::map<std::string, double> e6axis;
  std::string kuka_joints[12] = { "A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6" };

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    tmp[joint_names[i]] = point.positions[i] * RAD2DEG;
  }

  for (std::size_t i = 0; i < num_joints_; i++)
  {
    e6axis[kuka_joints[i]] == tmp[joint_names_[i]];
  }

    return e6axis;
}

bool KVPJointTrajectoryInterface::isCurrentMotion(double *wanted)
{
  double actual[12];
  std::string read_from = "$AXIS_FOR";

  if (!robot_.readE6AXIS(&read_from, actual))
  {
    ROS_ERROR("Error reading from robot");
    return false;
  }
 
  // Do a rough compare
  for (size_t i = 0; i < num_joints_; i++) {
     if ( (actual[i] > wanted[i] + 2) || (actual[i] < wanted[i] - 2) ) {
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
