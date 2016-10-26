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
#ifndef DYNAMIC_RECONFIGURE_SQLITE_STORAGE_SQLITE_STORAGE_H
#define DYNAMIC_RECONFIGURE_SQLITE_STORAGE_SQLITE_STORAGE_H

#include <dynamic_reconfigure/base_storage.h>


namespace dynamic_reconfigure_sqlite_storage
{

class SqliteStorage: public dynamic_reconfigure::BaseStorage
{
public:
  SqliteStorage();
  ~SqliteStorage();

  void initialize(std::string node_name, std::string storage_url);
  void loadConfig(dynamic_reconfigure::Config &msg);
  void saveConfig(const dynamic_reconfigure::Config &msg);

private:
  std::string db_path_;
  std::string table_name_;
  std::string node_name_;
  bool initialized_;
};

}  // namespace dynamic_reconfigure_sqlite_storage

#endif  // DYNAMIC_RECONFIGURE_SQLITE_STORAGE_SQLITE_STORAGE_H
