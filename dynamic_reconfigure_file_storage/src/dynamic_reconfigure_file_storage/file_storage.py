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
try:
    from urllib.parse import urlparse
    from urllib.request import url2pathname
except Exception:
    from urlparse import urlparse
    from urllib import url2pathname


class FileStorage(BaseStorage):

    def __init__(self, node_name, storage_url):
        super(FileStorage, self).__init__(node_name, storage_url)
        self.initialized = False

        protocol = 'file:///'
        if not storage_url.startswith(protocol):
            rospy.logerr('File storage URL must start with "file:///". Storage backend is not used.')
            self.initialized = False
        else:
            self.file_path = os.path.join(
                url2pathname(urlparse(storage_url).path),
                node_name.lstrip(os.path.sep),
                'params.yaml')
            self.initialized = True

    def load_config(self, msg):
        if not self.initialized:
            return None

        if os.path.isfile(self.file_path):
            yaml_str = None
            try:
                with open(self.file_path, 'r') as f:
                    yaml_str = f.read()
            except Exception as ex:
                rospy.logerr('Filesystem error: %s', ex)

            if yaml_str:
                return deserializeFromYaml(yaml_str)

        return None

    def save_config(self, msg):
        if not self.initialized:
            return

        dir_path = os.path.dirname(self.file_path)
        try:
            os.makedirs(dir_path)
        except OSError as ex:
            if not os.path.isdir(dir_path):
                rospy.logerr('Filesystem error: %s', ex)
                return

        try:
            with open(self.file_path, 'w') as f:
                yaml_str = serializeToYaml(msg)
                if yaml_str:
                    f.write(yaml_str)
        except Exception as ex:
            rospy.logerr('Filesystem error: %s', ex)
