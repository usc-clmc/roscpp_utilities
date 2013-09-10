/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2011-2013, CLMC Lab, University of Southern California
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */


#ifndef ROSCPP_UTILITIES_PARAM_H
#define ROSCPP_UTILITIES_PARAM_H

#include <string>
#include <vector>
#include <sstream>

#include <ros/node_handle.h>
#include <ros/assert.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Core>

namespace roscpp_utilities
{

// basic function overloaded for different types
bool getParam(XmlRpc::XmlRpcValue& config, int& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, long& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, unsigned int& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, unsigned long& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, float& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, double& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, bool& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, std::string& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Point& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Vector3& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Quaternion& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Pose& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Wrench& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, Eigen::VectorXd& value, const bool verbose=false, const std::string& ns="N/A");
bool getParam(XmlRpc::XmlRpcValue& config, Eigen::MatrixXd& value, const bool verbose=false, const std::string& ns="N/A");

// different kinds of convenient templated forms
template<class T>
bool getParam(XmlRpc::XmlRpcValue& config,
              const std::string& key,
              T& value,
              const bool verbose=false,
              const std::string& ns="N/A");

template<class T>
bool getParam(XmlRpc::XmlRpcValue& config,
              std::vector<T>& array,
              const bool verbose=false,
              const std::string& ns="N/A");

template<class T>
bool read(ros::NodeHandle& node_handle,
          const std::string& parameter_name,
          T& value,
          const bool verbose=true);

template<class T>
void require(ros::NodeHandle& node_handle,
             const std::string& parameter_name,
             T& value);

// other misc functions
void tokenizeString(const std::string& str_array, std::vector<std::string>& array);
bool readStringArraySpaceSeparated(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& array, const bool verbose = true);
void appendLeadingSlash(std::string& name);
void appendTrailingSlash(std::string& directory_name);
void removeLeadingSlash(std::string& topic_name);
std::string getString(const int number);
bool write(ros::NodeHandle& node_handle, const std::string& key,
           const std::vector<std::string>& str_array, const bool verbose=true);

} // namespace roscpp_utilities
#endif /* ROSCPP_UTILITIES_PARAM_H */
