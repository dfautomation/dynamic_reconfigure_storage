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

from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, StrParameter, DoubleParameter
import yaml

__all__ = ['serializeToYaml', 'deserializeFromYaml']


def serializeToYaml(msg):
    if not isinstance(msg, Config):
        return ''

    cfg = {
        'bools': [{'name': p.name, 'value': p.value} for p in msg.bools],
        'ints': [{'name': p.name, 'value': p.value} for p in msg.ints],
        'strs': [{'name': p.name, 'value': p.value} for p in msg.strs],
        'doubles': [{'name': p.name, 'value': p.value} for p in msg.doubles],
        'groups': [{'name': p.name, 'state': p.state, 'id': p.id, 'parent': p.parent} for p in msg.groups]
    }
    return yaml.dump(cfg)


def deserializeFromYaml(yaml_str):
    cfg = yaml.load(yaml_str)
    if not isinstance(cfg, dict):
        return None

    msg = Config()

    if 'bools' in cfg and isinstance(cfg['bools'], list):
        for p in cfg['bools']:
            try:
                msg.bools.append(BoolParameter(p['name'], p['value']))
            except:
                pass

    if 'ints' in cfg and isinstance(cfg['ints'], list):
        for p in cfg['ints']:
            try:
                msg.ints.append(IntParameter(p['name'], p['value']))
            except:
                pass

    if 'strs' in cfg and isinstance(cfg['strs'], list):
        for p in cfg['strs']:
            try:
                msg.strs.append(StrParameter(p['name'], p['value']))
            except:
                pass

    if 'doubles' in cfg and isinstance(cfg['doubles'], list):
        for p in cfg['doubles']:
            try:
                msg.doubles.append(DoubleParameter(p['name'], p['value']))
            except:
                pass

    return msg
