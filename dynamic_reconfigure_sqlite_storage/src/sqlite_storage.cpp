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

#include <dynamic_reconfigure_sqlite_storage/sqlite_storage.h>

#include <boost/filesystem.hpp>
#include <dynamic_reconfigure_storage_utils/utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sqlite3.h>

namespace fs = boost::filesystem;


// register as dynamic_reconfigure::BaseStorage plugin
PLUGINLIB_EXPORT_CLASS(dynamic_reconfigure_sqlite_storage::SqliteStorage, dynamic_reconfigure::BaseStorage)

namespace dynamic_reconfigure_sqlite_storage
{

SqliteStorage::SqliteStorage()
  : initialized_(false)
{
}

SqliteStorage::~SqliteStorage()
{
}

void SqliteStorage::initialize(std::string node_name, std::string storage_url)
{
  std::string protocol = "sqlite:///";
  if (storage_url.substr(0, protocol.length()) != protocol)
  {
    ROS_ERROR("SQLite storage URL must start with \"sqlite:///\". Storage backend is not used.");
    initialized_ = false;
  }
  else
  {
    size_t start = protocol.length() - 1;  // copy the slash too
    size_t hash_start = storage_url.find("#");
    db_path_ = storage_url.substr(start, hash_start);

    if (hash_start != std::string::npos)
    {
      table_name_ = storage_url.substr(hash_start + 1);
    }
    else
    {
      table_name_ = "dynamic_reconfigure_parameter";
    }

    node_name_ = node_name;
    initialized_ = true;
  }
}

void SqliteStorage::loadConfig(dynamic_reconfigure::Config &msg)
{
  if (!initialized_) return;

  try
  {
    fs::path path(db_path_);
    if (!fs::is_regular_file(path)) return;
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("Filesystem error: %s", ex.what());
    return;
  }

  sqlite3 *db = NULL;
  sqlite3_stmt *stmt = NULL;

  do
  {
    if (SQLITE_OK != sqlite3_open(db_path_.c_str(), &db))
    {
      ROS_ERROR("Can't open SQLite database: %s", sqlite3_errmsg(db));
      break;
    }

    std::string query = "SELECT value FROM `" + table_name_ + "` WHERE `key` = (?)";
    if (SQLITE_OK != sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, NULL))
    {
      ROS_ERROR("Can't prepare SQLite statement: %s", sqlite3_errmsg(db));
      break;
    }

    if (SQLITE_OK != sqlite3_bind_text(stmt, 1, node_name_.c_str(), -1, SQLITE_STATIC))
    {
      ROS_ERROR("Can't bind text: %s", sqlite3_errmsg(db));
      break;
    }

    bool once = false;
    while (true)
    {
      int s = sqlite3_step(stmt);
      if (s == SQLITE_ROW)
      {
        if (!once)
        {
          const char *text = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
          int len = sqlite3_column_bytes(stmt, 0);
          std::string yaml(text, text + len);
          dynamic_reconfigure_storage_utils::deserializeFromYaml(msg, yaml);
          once = true;
        }
        else
        {
          ROS_WARN("Duplicate row for node '%s'.", node_name_.c_str());
        }
      }
      else if (s == SQLITE_DONE)
      {
        break;
      }
      else
      {
        ROS_ERROR("Can't execute SELECT statement: %s", sqlite3_errmsg(db));
        break;
      }
    }
  }
  while (0);

  if (stmt) sqlite3_finalize(stmt);
  if (db) sqlite3_close(db);
}

void SqliteStorage::saveConfig(const dynamic_reconfigure::Config &msg)
{
  if (!initialized_) return;

  try
  {
    fs::path path(db_path_);
    fs::create_directories(path.parent_path());
  }
  catch (fs::filesystem_error &ex)
  {
    ROS_ERROR("Filesystem error: %s", ex.what());
    return;
  }

  sqlite3 *db = NULL;
  sqlite3_stmt *stmt = NULL;

  do
  {
    if (SQLITE_OK != sqlite3_open(db_path_.c_str(), &db))
    {
      ROS_ERROR("Can't open SQLite database: %s", sqlite3_errmsg(db));
      break;
    }

    std::string create_table = "CREATE TABLE IF NOT EXISTS `" + table_name_ + "` (" +
                               "`key` VARCHAR(255) PRIMARY KEY NOT NULL," +
                               "`value` TEXT NOT NULL)";

    if (SQLITE_OK != sqlite3_exec(db, create_table.c_str(), NULL, NULL, NULL))
    {
      ROS_ERROR("Can't create table: %s", sqlite3_errmsg(db));
      break;
    }

    std::string query = "INSERT OR REPLACE INTO `" + table_name_ + "` VALUES (?, ?)";
    if (SQLITE_OK != sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, NULL))
    {
      ROS_ERROR("Can't prepare SQLite statement: %s", sqlite3_errmsg(db));
      break;
    }
    if (SQLITE_OK != sqlite3_bind_text(stmt, 1, node_name_.c_str(), -1, SQLITE_STATIC))
    {
      ROS_ERROR("Can't bind text: %s", sqlite3_errmsg(db));
      break;
    }

    std::string yaml;
    dynamic_reconfigure_storage_utils::serializeToYaml(msg, yaml);
    if (SQLITE_OK != sqlite3_bind_text(stmt, 2, yaml.c_str(), -1, SQLITE_STATIC))
    {
      ROS_ERROR("Can't bind text: %s", sqlite3_errmsg(db));
      break;
    }

    if (SQLITE_DONE != sqlite3_step(stmt))
    {
      ROS_ERROR("Can't execute INSERT statement: %s", sqlite3_errmsg(db));
      break;
    }
  }
  while (0);

  if (stmt) sqlite3_finalize(stmt);
  if (db) sqlite3_close(db);
}

}  // namespace dynamic_reconfigure_sqlite_storage
