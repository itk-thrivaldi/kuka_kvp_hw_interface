/**
Copyright 2018 Mathias Hauan Arbo

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.
*/

/* Author: Mathias Hauan Arbo
   Desc: Read and write from the global variables on the KRC using
   KUKAVARPROXY (KVP). Currently only supports Booleans, but can be
   expanded.
*/

#ifndef KVP_VARIABLE_INTERFACE_H
#define KVP_VARIABLE_INTERFACE_H

// C++
#include <string>
#include <vector>

// ROS
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <industrial_msgs/TriState.h>
#include <industrial_msgs/ServiceReturnCode.h>
#include <kuka_kvp_hw_interface/SetBool.h>
#include <kuka_kvp_hw_interface/GetBool.h>
#include <kuka_kvp_hw_interface/SetInt.h>
#include <kuka_kvp_hw_interface/GetInt.h>
#include <kuka_kvp_hw_interface/SetFloat.h>
#include <kuka_kvp_hw_interface/GetFloat.h>

// KVP Communication
#include <BoostClientCross.h>

namespace kuka_kvp_hw_interface {
  /**
   * @brief Get or set variables on the KRC using KUKAVARPROXY
   */
  class KVPVariableInterface {
    protected:
    ros::NodeHandle nh_;

    // Variables relating to KVP configuration
    std::string ip_;

    /**
     * @brief Private method for sending boolean variable to robot
     * 
     * @param name, value
     *
     * @return True on success 
     */
    bool writeBool(const std::string& name, bool value);

    /**
     * @brief Private method for sending int variable to robot 
     * 
     *
     * @param name, value
     *
     * @return True on success (TODO)
     */
    bool writeInt(const std::string& name, int value);

    /**
     * @brief Private method for sending float variable to robot 
     *
     * @param name, value
     *
     * @return True on success (TODO)
     */
    bool writeFloat(const std::string& name, double value);
    
  public:
    KVPVariableInterface(ros::NodeHandle& nh);

    /** \brief Get state of boolean in KRC
     */
    bool getBool(kuka_kvp_hw_interface::GetBool::Request& req,
		 kuka_kvp_hw_interface::GetBool::Response& res);

    /** \brief Set state of boolean in KRC
     */
    bool setBool(kuka_kvp_hw_interface::SetBool::Request& req,
		 kuka_kvp_hw_interface::SetBool::Response& res);
    /** \brief Get int in KRC
     */
    bool getInt(kuka_kvp_hw_interface::GetInt::Request& req,
	        kuka_kvp_hw_interface::GetInt::Response& res);
    /** \brief Set int in KRC. Only Int32.
     */
    bool setInt(kuka_kvp_hw_interface::SetInt::Request& req,
		kuka_kvp_hw_interface::SetInt::Response& res);
    /** \brief Get float in KRC. Only float32.
     */
    bool getFloat(kuka_kvp_hw_interface::GetFloat::Request& req,
		  kuka_kvp_hw_interface::GetFloat::Response& res);
    /** \brief Set float in KRC. Only float32.
     */
    bool setFloat(kuka_kvp_hw_interface::SetFloat::Request& req,
		  kuka_kvp_hw_interface::SetFloat::Response& res);
    
  };
}

#endif //KVP_VARIABLE_INTERFACE_H
