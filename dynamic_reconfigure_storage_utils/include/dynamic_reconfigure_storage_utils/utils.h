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
 *   * Neither the name of DF Automation & Robotics nor the names of its
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

#ifndef DYNAMIC_RECONFIGURE_STORAGE_UTILS_UTILS_H
#define DYNAMIC_RECONFIGURE_STORAGE_UTILS_UTILS_H

#include <dynamic_reconfigure/Config.h>
#include <yaml-cpp/yaml.h>


namespace dynamic_reconfigure_storage_utils
{
// Python-compatible YAML serialization.
void serializeToYaml(const dynamic_reconfigure::Config &msg, std::string &yaml)
{
  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "bools" << YAML::Value << YAML::BeginSeq;
  for (std::vector<dynamic_reconfigure::BoolParameter>::const_iterator it = msg.bools.begin(); it != msg.bools.end(); ++it)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << it->name;
    out << YAML::Key << "value" << YAML::Value << static_cast<bool>(it->value);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "ints" << YAML::Value << YAML::BeginSeq;
  for (std::vector<dynamic_reconfigure::IntParameter>::const_iterator it = msg.ints.begin(); it != msg.ints.end(); ++it)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << it->name;
    out << YAML::Key << "value" << YAML::Value << it->value;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "strs" << YAML::Value << YAML::BeginSeq;
  for (std::vector<dynamic_reconfigure::StrParameter>::const_iterator it = msg.strs.begin(); it != msg.strs.end(); ++it)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << it->name;
    out << YAML::Key << "value" << YAML::Value << it->value;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "doubles" << YAML::Value << YAML::BeginSeq;
  for (std::vector<dynamic_reconfigure::DoubleParameter>::const_iterator it = msg.doubles.begin(); it != msg.doubles.end(); ++it)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << it->name;
    out << YAML::Key << "value" << YAML::Value << it->value;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "groups" << YAML::Value << YAML::BeginSeq;
  for (std::vector<dynamic_reconfigure::GroupState>::const_iterator it = msg.groups.begin(); it != msg.groups.end(); ++it)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << it->name;
    out << YAML::Key << "state" << YAML::Value << static_cast<bool>(it->state);
    out << YAML::Key << "id" << YAML::Value << it->id;
    out << YAML::Key << "parent" << YAML::Value << it->parent;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::EndMap;

  yaml = out.c_str();
}

// Python-compatible YAML deserialization.
void deserializeFromYaml(dynamic_reconfigure::Config &msg, const std::string &yaml)
{
  YAML::Node node = YAML::Load(yaml);
  if (node.IsNull()) return;
  if (!node.IsMap()) return;

  if (node["bools"].IsSequence())
  {
    msg.bools.clear();
    dynamic_reconfigure::BoolParameter param;
    for (YAML::const_iterator it = node["bools"].begin(); it != node["bools"].end(); ++it)
    {
      if (it->IsMap() && (*it)["name"] && (*it)["value"])
      {
        param.name = (*it)["name"].as<std::string>();
        param.value = (*it)["value"].as<bool>();
        msg.bools.push_back(param);
      }
    }
  }

  if (node["ints"].IsSequence())
  {
    msg.ints.clear();
    dynamic_reconfigure::IntParameter param;
    for (YAML::const_iterator it = node["ints"].begin(); it != node["ints"].end(); ++it)
    {
      if (it->IsMap() && (*it)["name"] && (*it)["value"])
      {
          param.name = (*it)["name"].as<std::string>();
          param.value = (*it)["value"].as<int32_t>();
        msg.ints.push_back(param);
      }
    }
  }

  if (node["strs"].IsSequence())
  {
    msg.strs.clear();
    dynamic_reconfigure::StrParameter param;
    for (YAML::const_iterator it = node["strs"].begin(); it != node["strs"].end(); ++it)
    {
      if (it->IsMap() && (*it)["name"] && (*it)["value"])
      {
        param.name = (*it)["name"].as<std::string>();
        param.value = (*it)["value"].as<std::string>();
        msg.strs.push_back(param);
      }
    }
  }

  if (node["doubles"].IsSequence())
  {
    msg.doubles.clear();
    dynamic_reconfigure::DoubleParameter param;
    for (YAML::const_iterator it = node["doubles"].begin(); it != node["doubles"].end(); ++it)
    {
      if (it->IsMap() && (*it)["name"] && (*it)["value"])
      {
        param.name = (*it)["name"].as<std::string>();
        param.value = (*it)["value"].as<double>();
        msg.doubles.push_back(param);
      }
    }
  }

  if (node["groups"].IsSequence())
  {
    msg.groups.clear();
    dynamic_reconfigure::GroupState group;
    for (YAML::const_iterator it = node["groups"].begin(); it != node["groups"].end(); ++it)
    {
      if (it->IsMap() && (*it)["name"] && (*it)["state"] && (*it)["id"] && (*it)["parent"])
      {
        group.name = (*it)["name"].as<std::string>();
        group.state = (*it)["state"].as<bool>();
        group.id = (*it)["id"].as<int32_t>();
        group.parent = (*it)["parent"].as<int32_t>();
        msg.groups.push_back(group);
      }
    }
  }
}

}  // namespace dynamic_reconfigure_storage_utils

#endif  // DYNAMIC_RECONFIGURE_STORAGE_UTILS_UTILS_H
