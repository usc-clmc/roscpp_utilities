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

#include <roscpp_utilities/param_server.h>

namespace roscpp_utilities
{

template<class T>
bool getParam(XmlRpc::XmlRpcValue& config,
              const std::string& key,
              T& value,
              const bool verbose,
              const std::string& ns)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR_COND(verbose, "Parameter %s does not have member %s.", ns.c_str(), key.c_str());
    return false;
  }

  return getParam(config[key], value, verbose, ns);
}

template<class T>
bool getParam(XmlRpc::XmlRpcValue& config,
              std::vector<T>& array,
              const bool verbose,
              const std::string& ns)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_COND(verbose, "Parameter %s must be an array.", ns.c_str());
    return false;
  }

  int size = config.size();
  array.resize(size);
  for (int i=0; i<size; ++i)
  {
    if (!getParam(config[i], static_cast<T&>(array[i]), verbose, ns))
    {
      ROS_ERROR_COND(verbose, "Error reading parameter %s[%d]", ns.c_str(), i);
      return false;
    }
  }

  return true;
}

template<class T>
bool read(ros::NodeHandle& node_handle,
          const std::string& parameter_name,
          T& value,
          const bool verbose)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  const std::string full_ns = node_handle.getNamespace() + "/" + parameter_name;
  if (!node_handle.getParam(parameter_name, xml_rpc_value))
  {
    ROS_ERROR_COND(verbose, "Couldn't read parameter %s.", full_ns.c_str());
    return false;
  }

  return getParam(xml_rpc_value, value, verbose, full_ns);
}

template<class T>
void require(ros::NodeHandle& node_handle,
             const std::string& parameter_name,
             T& value)
{
  if (!read(node_handle, parameter_name, value, true))
  {
    ROS_BREAK();
  }
}

// overloaded basic functions

bool getParam(XmlRpc::XmlRpcValue& config, int& value, const bool verbose, const std::string& ns)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR_COND(verbose, "Parameter %s must be of type int.", ns.c_str());
    return false;
  }

  value = static_cast<int>(config);
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, long& value, const bool verbose, const std::string& ns)
{
  int int_value;
  if (!getParam(config, int_value, verbose, ns))
    return false;

  value = int_value;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, unsigned long& value, const bool verbose, const std::string& ns)
{
  int int_value;
  if (!getParam(config, int_value, verbose, ns))
    return false;

  value = int_value;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, unsigned int& value, const bool verbose, const std::string& ns)
{
  int int_value;
  if (!getParam(config, int_value, verbose, ns))
    return false;

  value = int_value;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, double& value, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    value = static_cast<double>(config);
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeInt)
    value = static_cast<double>(static_cast<int>(config));
  else
  {
    ROS_ERROR_COND(verbose, "Parameter %s must be of type double or int.", ns.c_str());
    return false;
  }

  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, float& value, const bool verbose, const std::string& ns)
{
  double double_value;
  if (!getParam(config, double_value, verbose, ns))
    return false;

  value = double_value;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, bool& value, const bool verbose, const std::string& ns)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    ROS_ERROR_COND(verbose, "Parameter %s must be of type bool.", ns.c_str());
    return false;
  }

  value = static_cast<bool>(config);
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, std::string& value, const bool verbose, const std::string& ns)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_ERROR_COND(verbose, "Parameter %s must be of type string.", ns.c_str());
    return false;
  }

  value = static_cast<std::string>(config);
  return true;
}

// geometry_msgs functions

bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Point& position, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (config.size() == 3 &&
        getParam(config[0], position.x, verbose, ns+"[0]") &&
        getParam(config[1], position.y, verbose, ns+"[1]") &&
        getParam(config[2], position.z, verbose, ns+"[2]"))
    {
      return true;
    }
  }
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (config.hasMember("x") &&
        config.hasMember("y") &&
        config.hasMember("z") &&
        getParam(config["x"], position.x, verbose, ns+"/x") &&
        getParam(config["y"], position.y, verbose, ns+"/y") &&
        getParam(config["z"], position.z, verbose, ns+"/z"))
    {
      return true;
    }
  }

  ROS_ERROR_COND(verbose, "Parameter %s must either be an array of 3 numbers or a struct containing x, y, z.", ns.c_str());
  return false;
}

bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Vector3& position, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (config.size() == 3 &&
        getParam(config[0], position.x, verbose, ns+"[0]") &&
        getParam(config[1], position.y, verbose, ns+"[1]") &&
        getParam(config[2], position.z, verbose, ns+"[2]"))
    {
      return true;
    }
  }
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (config.hasMember("x") &&
        config.hasMember("y") &&
        config.hasMember("z") &&
        getParam(config["x"], position.x, verbose, ns+"/x") &&
        getParam(config["y"], position.y, verbose, ns+"/y") &&
        getParam(config["z"], position.z, verbose, ns+"/z"))
    {
      return true;
    }
  }

  ROS_ERROR_COND(verbose, "Parameter %s must either be an array of 3 numbers or a struct containing x, y, z.", ns.c_str());
  return false;
}

bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Quaternion& quat, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (config.size() == 4 &&
        getParam(config[0], quat.w, verbose, ns+"[0]") &&
        getParam(config[1], quat.x, verbose, ns+"[1]") &&
        getParam(config[2], quat.y, verbose, ns+"[2]") &&
        getParam(config[3], quat.z, verbose, ns+"[3]"))
    {
      return true;
    }
  }
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (config.hasMember("w") &&
        config.hasMember("x") &&
        config.hasMember("y") &&
        config.hasMember("z") &&
        getParam(config["w"], quat.w, verbose, ns+"/w") &&
        getParam(config["x"], quat.x, verbose, ns+"/x") &&
        getParam(config["y"], quat.y, verbose, ns+"/y") &&
        getParam(config["z"], quat.z, verbose, ns+"/z"))
    {
      return true;
    }
  }


  ROS_ERROR_COND(verbose, "Parameter %s must either be an array of 4 numbers or a struct containing w, x, y, z.", ns.c_str());
  return false;
}

bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Pose& pose, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (config.size() == 7 &&
        getParam(config[0], pose.position.x, verbose, ns+"[0]") &&
        getParam(config[1], pose.position.y, verbose, ns+"[1]") &&
        getParam(config[2], pose.position.z, verbose, ns+"[2]") &&
        getParam(config[3], pose.orientation.w, verbose, ns+"[3]") &&
        getParam(config[4], pose.orientation.x, verbose, ns+"[4]") &&
        getParam(config[5], pose.orientation.y, verbose, ns+"[5]") &&
        getParam(config[6], pose.orientation.z, verbose, ns+"[6]"))
    {
      return true;
    }
  }
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (config.hasMember("position") &&
        config.hasMember("orientation") &&
        getParam(config["position"], pose.position, verbose, ns+"/position") &&
        getParam(config["orientation"], pose.orientation, verbose, ns+"/orientation"))
    {
      return true;
    }
  }


  ROS_ERROR_COND(verbose, "Parameter %s must either be an array of 7 numbers or a struct containing position and orientation.", ns.c_str());
  return false;
}

bool getParam(XmlRpc::XmlRpcValue& config, geometry_msgs::Wrench& wrench, const bool verbose, const std::string& ns)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (config.size() == 6 &&
        getParam(config[0], wrench.force.x, verbose, ns+"[0]") &&
        getParam(config[1], wrench.force.y, verbose, ns+"[1]") &&
        getParam(config[2], wrench.force.z, verbose, ns+"[2]") &&
        getParam(config[3], wrench.torque.x, verbose, ns+"[3]") &&
        getParam(config[4], wrench.torque.y, verbose, ns+"[4]") &&
        getParam(config[5], wrench.torque.z, verbose, ns+"[5]"))
    {
      return true;
    }
  }
  else if (config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (config.hasMember("force") &&
        config.hasMember("torque") &&
        getParam(config["force"], wrench.force, verbose, ns+"/force") &&
        getParam(config["torque"], wrench.torque, verbose, ns+"/torque"))
    {
      return true;
    }
  }


  ROS_ERROR_COND(verbose, "Parameter %s must either be an array of 6 numbers or a struct containing force and torque.", ns.c_str());
  return false;
}

bool getParam(XmlRpc::XmlRpcValue& config, Eigen::VectorXd& vector, const bool verbose, const std::string& ns)
{
  std::vector<double> array;
  if (!getParam(config, array, verbose, ns))
    return false;

  vector=Eigen::VectorXd(array.size());
  for (size_t i=0; i<array.size(); ++i)
    vector(i) = array[i];
  return true;
}

// legacy functions

void tokenizeString(const std::string& str_array, std::vector<std::string>& array)
{
  array.clear();
  std::stringstream str_stream(str_array);
  std::string item;
  while (str_stream >> item)
  {
    array.push_back(item);
  }
}

bool readStringArraySpaceSeparated(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& array, const bool verbose)
{
  std::string str_array;

  if (!node_handle.getParam(parameter_name, str_array))
  {
    ROS_ERROR_COND(verbose, "Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
    return false;
  }

  tokenizeString(str_array, array);
  return true;
}

void appendLeadingSlash(std::string& name)
{
  if (name.compare(0, 1, "/") != 0) // the name does not start with a slash
  {
    name = std::string("/") + name;
  }
}

void appendTrailingSlash(std::string& directory_name)
{
  if (directory_name.compare(directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
  {
    directory_name.append("/");
  }
}

void removeLeadingSlash(std::string& topic_name)
{
  if (topic_name.compare(0, 1, "/") == 0) // the topic name starts with a slash
  {
    topic_name.assign(topic_name.substr(1, topic_name.length()));
  }
}

std::string getString(const int number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

// create specific instantiations of templated functions, for the types that we know will work

#define DEFINE_PS_SPECIALIZATIONS(PS_TYPE) \
    template bool getParam<PS_TYPE>(XmlRpc::XmlRpcValue& config, const std::string& key, PS_TYPE& value, const bool verbose, const std::string& ns);\
    template bool getParam<std::vector<PS_TYPE> >(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<PS_TYPE>& value, const bool verbose, const std::string& ns);\
    template bool getParam<PS_TYPE>(XmlRpc::XmlRpcValue& config, std::vector<PS_TYPE>& array, const bool verbose, const std::string& ns); \
    template bool read<PS_TYPE>(ros::NodeHandle& node_handle, const std::string& parameter_name, PS_TYPE& value, const bool verbose); \
    template bool read<std::vector<PS_TYPE> >(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<PS_TYPE>& value, const bool verbose); \
    template void require<PS_TYPE>(ros::NodeHandle& node_handle, const std::string& parameter_name, PS_TYPE& value); \
    template void require<std::vector<PS_TYPE> >(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<PS_TYPE>& value); \

DEFINE_PS_SPECIALIZATIONS(int)
DEFINE_PS_SPECIALIZATIONS(long)
DEFINE_PS_SPECIALIZATIONS(unsigned int)
DEFINE_PS_SPECIALIZATIONS(unsigned long)
DEFINE_PS_SPECIALIZATIONS(float)
DEFINE_PS_SPECIALIZATIONS(double)
DEFINE_PS_SPECIALIZATIONS(std::string)
DEFINE_PS_SPECIALIZATIONS(geometry_msgs::Point)
DEFINE_PS_SPECIALIZATIONS(geometry_msgs::Vector3)
DEFINE_PS_SPECIALIZATIONS(geometry_msgs::Quaternion)
DEFINE_PS_SPECIALIZATIONS(geometry_msgs::Pose)
DEFINE_PS_SPECIALIZATIONS(geometry_msgs::Wrench)
DEFINE_PS_SPECIALIZATIONS(Eigen::VectorXd)

// specialize separately for bool because std::vector<bool> is non-standard. Grrr....
template bool getParam<bool>(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value, const bool verbose, const std::string& ns);
template bool read<bool>(ros::NodeHandle& node_handle, const std::string& parameter_name, bool& value, const bool verbose);
template void require<bool>(ros::NodeHandle& node_handle, const std::string& parameter_name, bool& value);


} // namespace roscpp_utilities
