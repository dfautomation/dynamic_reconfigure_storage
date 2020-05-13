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

#include <dynamic_reconfigure_mysql_storage/mysql_storage.h>

#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <dynamic_reconfigure_storage_utils/utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


// register as dynamic_reconfigure::BaseStorage plugin
PLUGINLIB_EXPORT_CLASS(dynamic_reconfigure_mysql_storage::MysqlStorage, dynamic_reconfigure::BaseStorage)

namespace dynamic_reconfigure_mysql_storage
{

MysqlStorage::MysqlStorage()
  : initialized_(false)
{
}

MysqlStorage::~MysqlStorage()
{
}

void MysqlStorage::initialize(std::string node_name, std::string storage_url)
{
  initialized_ = false;
  std::string protocol = "mysql://";
  if (storage_url.substr(0, protocol.length()) != protocol)
  {
    ROS_ERROR("MySQL storage URL must start with \"mysql://\". Storage backend is not used.");
    return;
  }

  storage_url = storage_url.substr(protocol.length());
  size_t db_schema_start = storage_url.find("/");
  if (db_schema_start == std::string::npos)
  {
    ROS_ERROR("Database schema name must be specified in the storage url after the host name, separated by a slash.");
    return;
  }

  db_schema_ = storage_url.substr(db_schema_start + 1);
  storage_url = storage_url.substr(0, db_schema_start);

  size_t table_name_start = db_schema_.find('#');
  if (table_name_start != std::string::npos)
  {
    table_name_ = db_schema_.substr(table_name_start + 1);
    db_schema_ = db_schema_.substr(0, table_name_start);
  }
  else
  {
    table_name_ = "dynamic_reconfigure_parameter";
  }

  size_t db_host_start = storage_url.find('@');
  if (db_host_start != std::string::npos)
  {
    db_host_ = storage_url.substr(db_host_start + 1);
    storage_url = storage_url.substr(0, db_host_start);

    size_t db_pw_start = storage_url.find(':');
    if (db_pw_start != std::string::npos)
    {
      db_user_ = storage_url.substr(0, db_pw_start);
      db_password_ = storage_url.substr(db_pw_start + 1);
    }
    else
    {
      db_user_ = storage_url;
      db_password_ = "";
    }
  }
  else
  {
    db_host_ = storage_url;
    db_user_ = "";
    db_password_ = "";
  }

  node_name_ = node_name;
  initialized_ = true;
}

void MysqlStorage::loadConfig(dynamic_reconfigure::Config & msg)
{
  if (!initialized_) return;

  sql::Driver *driver = NULL;
  sql::Connection *conn = NULL;
  sql::PreparedStatement *pstmt = NULL;
  sql::ResultSet *res = NULL;

  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string query = "SELECT value FROM `" + table_name_ + "` WHERE `key` = (?)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, node_name_);
    res = pstmt->executeQuery();

    bool once = false;
    while (res->next())
    {
      if (!once)
      {
        std::string yaml = res->getString(1);
        dynamic_reconfigure_storage_utils::deserializeFromYaml(msg, yaml);
        once = true;
      }
      else
      {
        ROS_WARN("Duplicate row for node '%s'.", node_name_.c_str());
      }
    }
  }
  catch (sql::SQLException &ex)
  {
    ROS_ERROR("Database error: %s", ex.what());
  }

  if (res) delete res;
  if (pstmt) delete pstmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
}

void MysqlStorage::saveConfig(const dynamic_reconfigure::Config & msg)
{
  if (!initialized_) return;

  sql::Driver *driver = NULL;
  sql::Connection *conn = NULL;
  sql::Statement *stmt = NULL;
  sql::PreparedStatement *pstmt = NULL;

  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string create_table = "CREATE TABLE IF NOT EXISTS `" + table_name_ + "` (" +
                               "`key` VARCHAR(255) PRIMARY KEY NOT NULL," +
                               "`value` TEXT NOT NULL)";
    stmt = conn->createStatement();
    stmt->execute(create_table.c_str());

    std::string query = "INSERT INTO `" + table_name_ + "` VALUES (?, ?)" +
                        "ON DUPLICATE KEY UPDATE `value`=(?)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, node_name_);

    std::string yaml;
    dynamic_reconfigure_storage_utils::serializeToYaml(msg, yaml);
    pstmt->setString(2, yaml);
    pstmt->setString(3, yaml);
    pstmt->executeUpdate();
  }
  catch (sql::SQLException &ex)
  {
    ROS_ERROR("Database error: %s", ex.what());
  }

  if (pstmt) delete pstmt;
  if (stmt) delete stmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
}

}  // namespace dynamic_reconfigure_mysql_storage
