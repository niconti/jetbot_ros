#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of {copyright_holder} nor the names of its
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
#
# Author: Nicola Conti

import re
import subprocess
# ROS
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from wireless_msgs.msg import Connection, Network
# UPS-Power-Module
from jetbot_ros.INA219 import INA219


class MonitorWifi(rclpy.node.Node):
 
    def __init__(self, ina219):
        super().__init__('monitor_wifi_node')
        self.ina219 = ina219

        # Parameters
        self.declare_parameter('frequency', 2, ParameterDescriptor(description="The frequency at which to monitor battery state"))
        self.declare_parameter('ifname', 'wlan0', ParameterDescriptor(description="The wifi interface to monior"))
        self.declare_parameter('warninig_level', 20, ParameterDescriptor(description="The amount of battery charge considered low"))
        self.declare_parameter('critical_level', 10, ParameterDescriptor(description="The amount of battery charge considered critical"))

        # Published Topics
        self.connection_pub = self.create_publisher(Connection, 'connection', 10)

        # Timers
        freq= self.get_parameter('frequency').value
        self.update_timer = self.create_timer(freq, self.update_cb)


    def update_cb(self):
        now = self.get_clock().now()

        IFNAME = self.get_parameter('ifname').value
        WARNING_LEVEL  = self.get_parameter('warninig_level').value
        CRITICAL_LEVEL = self.get_parameter('critical_level').value

        
        wifi_str = subprocess.check_output(["iwlist", IFNAME, "scan"], stderr=subprocess.STDOUT).decode()
        fields_str = re.split('\s\s+', wifi_str)
        fields_list = [re.split('[:=]', field_str, maxsplit=1) for field_str in fields_str]
        fields_dict = {'dev': fields_str[0], 'type': fields_str[1]}
        fields_dict.update(dict([field for field in fields_list if len(field) == 2]))
        print(wifi_str)


        # self.connection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)


    # Node
    node = MonitorWifi(ina219)

    # Spin
    rclpy.spin(node)


if __name__ == '__main__':
    main()
