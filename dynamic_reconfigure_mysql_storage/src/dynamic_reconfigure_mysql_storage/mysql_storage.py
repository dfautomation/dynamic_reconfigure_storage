# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
from sqlalchemy import Column, String, Text, create_engine
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import rospy

Base = declarative_base()


class MysqlStorage(BaseStorage):

    def __init__(self, node_name, storage_url):
        super(MysqlStorage, self).__init__(node_name, storage_url)
        self.initialized = False

        protocol = 'mysql://'
        if not storage_url.startswith(protocol):
            rospy.logerr('MySQL storage URL must start with "mysql://". Storage backend is not used.')
            self.initialized = False
        else:
            hash_start = storage_url.find('#')
            self.db_url = storage_url[:hash_start]

            table_name = 'dynamic_reconfigure_parameter'
            if hash_start != -1:
                table_name = storage_url[hash_start + 1:]

            self.Parameter = type('Parameter', (Base,), {
                '__tablename__': table_name,
                'key': Column(String(255), primary_key=True, nullable=False),
                'value': Column(Text, nullable=False),
            })

            self.node_name = node_name
            self.initialized = True

    def load_config(self, msg):
        if not self.initialized:
            return None

        msg = None
        try:
            engine = create_engine(self.db_url)
            session = sessionmaker(bind=engine)()

            param = session.query(self.Parameter).filter_by(key=self.node_name).first()
            if param:
                msg = deserializeFromYaml(param.value)

            session.commit()

        except SQLAlchemyError as ex:
            rospy.logerr('Database error: %s', str(ex))

        return msg

    def save_config(self, msg):
        if not self.initialized:
            return

        yaml_str = serializeToYaml(msg)
        if not yaml_str:
            return

        try:
            engine = create_engine(self.db_url)
            Base.metadata.create_all(engine, checkfirst=True)
            session = sessionmaker(bind=engine)()

            param = self.Parameter()
            param.key = self.node_name
            param.value = yaml_str
            session.merge(param)

            session.commit()

        except SQLAlchemyError as ex:
            rospy.logerr('Database error: %s', str(ex))
