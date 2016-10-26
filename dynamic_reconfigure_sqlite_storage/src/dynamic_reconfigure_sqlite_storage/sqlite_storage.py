# Software License Agreement (BSD License)
#
# Copyright (c) 2009, DF Automation & Robotics Sdn Bhd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of DF Automation & Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Patrick Chin

from dynamic_reconfigure.storage import BaseStorage
from dynamic_reconfigure_storage_utils.utils import *
import os
import rospy
import sqlite3
import urllib
import urlparse


class SqliteStorage(BaseStorage):

    def __init__(self, node_name, storage_url):
        super(SqliteStorage, self).__init__(node_name, storage_url)
        self.initialized = False

        protocol = 'sqlite:///'
        if not storage_url.startswith(protocol):
            rospy.logerr('SQLite storage URL must start with "sqlite:///". Storage backend is not used.')
            self.initialized = False
        else:
            u = urlparse.urlparse(storage_url)
            self.db_path = urllib.url2pathname(u.path)
            self.table_name = u.fragment or 'dynamic_reconfigure_parameter'
            self.node_name = node_name
            self.initialized = True

    def load_config(self, msg):
        if not self.initialized:
            return None

        if not os.path.isfile(self.db_path):
            return None

        msg = None
        try:
            conn = sqlite3.connect(self.db_path)
            with conn:
                once = False
                for row in conn.execute('SELECT value FROM `' + self.table_name + '`WHERE key = ?', (self.node_name, )):
                    if not once:
                        yaml_str = row[0]
                        msg = deserializeFromYaml(yaml_str)

            conn.close()

        except sqlite3.Error as ex:
            rospy.logerr('SQLite error: %s', str(ex))

        return msg

    def save_config(self, msg):
        if not self.initialized:
            return

        dir_path = os.path.dirname(self.db_path)
        try:
            os.makedirs(dir_path)
        except OSError as ex:
            if not os.path.isdir(dir_path):
                rospy.logerr('Filesystem error: %s', ex)
                return

        yaml_str = serializeToYaml(msg)
        if not yaml_str:
            return

        try:
            conn = sqlite3.connect(self.db_path)
            with conn:
                conn.execute('CREATE TABLE IF NOT EXISTS `' + self.table_name + '` (' +
                    '`key` VARCHAR(255) PRIMARY KEY NOT NULL,' +
                    '`value` TEXT NOT NULL)')
                conn.execute('INSERT OR REPLACE INTO `' + self.table_name + '` VALUES (?, ?)', (self.node_name, yaml_str))

            conn.close()

        except sqlite3.Error as ex:
            rospy.logerr('SQLite error: %s', str(ex))
