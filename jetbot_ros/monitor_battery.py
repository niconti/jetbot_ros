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

import os
import sys
import time
import string
# ROS
import rclpy
import rclpy.node
import rclpy.logging
import sensor_msgs.msg
from rcl_interfaces.msg import ParameterDescriptor
# UPS-Power-Module
from jetbot_ros.INA219 import INA219


class MonitorBattery(rclpy.node.Node):
 
    def __init__(self):
        super().__init__('monitor_battery_node')

        # Parameters
        self.declare_parameter('warninig_level', 20, ParameterDescriptor(description="The amount of battery charge considered low"))
        self.declare_parameter('critical_level', 2, ParameterDescriptor(description="The amount of battery charge considered critical"))

        # Published Topics
        self.battery_state_pub = self.create_publisher(sensor_msgs.msg.BatteryState, '/battery_state', 10)
      

def main(args=None):    
    rclpy.init(args=args)

    # Node
    node = MonitorBattery()

    # Parameters
    WARNING_LEVEL  = node.get_parameter('warninig_level').value
    CRITICAL_LEVEL = node.get_parameter('critical_level').value

    # Init INA219
    try:
        ina219 = INA219(addr=0x42)
    except OSError as ex:
        node.get_logger().fatal("{}".format(ex))
        exit(1)

    # rate = node.create_rate(10)
    while rclpy.ok():

        now = node.get_clock().now()

        bus_voltage   = ina219.getBusVoltage_V()            # voltage on V- (load side)
        shunt_voltage = ina219.getShuntVoltage_mV() / 1000  # voltage between V+ and V- across the shunt
        current = ina219.getCurrent_mA() / 1000             # current in A
        power = ina219.getPower_W()                         # power in W      
        percentage = (bus_voltage - 6.0) / 2.4
        percentage = max(percentage, 0)
        percentage = min(100, percentage)
        
        # message
        msg = sensor_msgs.msg.BatteryState()
        msg.header.stamp = now.to_msg()
        msg.voltage = bus_voltage
        msg.current = current
        msg.percentage = percentage
        msg.design_capacity = 12.000                         # 4 x 3000 mA
        
        if current > 0:
            msg.power_supply_status = sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_CHARGING 
        if current < 0:
            msg.power_supply_status = sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if percentage == 100.0:
            msg.power_supply_status = sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_FULL
        
        msg.power_supply_health = sensor_msgs.msg.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = sensor_msgs.msg.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        node.battery_state_pub.publish(msg)
        time.sleep(2)


if __name__ == '__main__':
    main()
