/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, DF Automation & Robotics Sdn Bhd.
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
 *
 * Author: Patrick Chin
 *********************************************************************/

#include <dynamic_reconfigure_file_storage/file_storage.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <dynamic_reconfigure_storage_utils/utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sstream>

namespace fs = boost::filesystem;


// register as dynamic_reconfigure::BaseStorage plugin
PLUGINLIB_EXPORT_CLASS(dynamic_reconfigure_file_storage::FileStorage, dynamic_reconfigure::BaseStorage)

namespace dynamic_reconfigure_file_storage
{

FileStorage::FileStorage()
  : initialized_(false)
{
}

FileStorage::~FileStorage()
{
}

void FileStorage::initialize(std::string node_name, std::string storage_url)
{
  std::string protocol = "file:///";
  if (storage_url.substr(0, protocol.length()) != protocol)
  {
    ROS_ERROR("File storage URL must start with \"file:///\". Storage backend is not used.");
    initialized_ = false;
  }
  else
  {
    size_t start = protocol.length() - 1;  // copy the slash too
    file_path_ = storage_url.substr(start) + (node_name + "/params.yaml");
    initialized_ = true;
  }
}

void FileStorage::loadConfig(dynamic_reconfigure::Config &msg)
{
  if (!initialized_) return;

  try
  {
    fs::path path(file_path_);
    if (fs::is_regular_file(path))
    {
      fs::ifstream ifs(path);

      std::ostringstream oss;
      oss << ifs.rdbuf();
      std::string yaml = oss.str();

      dynamic_reconfigure_storage_utils::deserializeFromYaml(msg, yaml);
    }
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("Filesystem error: %s", ex.what());
  }
}

void FileStorage::saveConfig(const dynamic_reconfigure::Config &msg)
{
  if (!initialized_) return;

  try
  {
    fs::path path(file_path_);
    fs::create_directories(path.parent_path());
    fs::ofstream ofs(path);

    std::string yaml;
    dynamic_reconfigure_storage_utils::serializeToYaml(msg, yaml);
    ofs << yaml;
  }
  catch (fs::filesystem_error &ex)
  {
    ROS_ERROR("Filesystem error: %s", ex.what());
  }
}

}  // namespace dynamic_reconfigure_file_storage
