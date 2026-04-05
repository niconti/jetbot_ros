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

import rclpy
import rclpy.logging
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import BatteryState
# Waveshare UPS
from jetbot_ros.INA219 import INA219


class MonitorBattery(Node):
    """
    Node monitoring battery charge level.
    """

    def __init__(self, ina219):
        super().__init__('monitor_battery_node')
        self.MAX_VOLTAGE = 8.4
        self.MIN_VOLTAGE = 6.0
        self.ina219 = ina219

        # Parameters
        self.declare_parameter('frequency', 2, ParameterDescriptor(description="The frequency at which to monitor battery state"))
        self.declare_parameter('warninig_level', 20, ParameterDescriptor(description="The amount of battery charge considered low"))
        self.declare_parameter('critical_level', 10, ParameterDescriptor(description="The amount of battery charge considered critical"))

        # Publishers
        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', 10)

        # Timers
        freq = self.get_parameter('frequency').value
        self.update_timer = self.create_timer(freq, self.update_cb)


    def update_cb(self):
        timestamp = self.get_clock().now()

        WARNING_LEVEL  = self.get_parameter('warninig_level').value
        CRITICAL_LEVEL = self.get_parameter('critical_level').value

        try:
            bus_voltage   = self.ina219.getBusVoltage_V()               # voltage on V- (load side)
            shunt_voltage = self.ina219.getShuntVoltage_mV() / 1000     # voltage between V+ and V- across the shunt
            current       = self.ina219.getCurrent_mA() / 1000          # current in A
            power         = self.ina219.getPower_W()                    # power in W      
        except OSError as e:
            self.get_logger().error("{}".format(e))
            return

        percentage = (bus_voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)
        percentage = max(0, percentage)
        percentage = min(1, percentage)

        # message
        msg = BatteryState()
        msg.header.stamp = timestamp.to_msg()
        msg.voltage = bus_voltage
        msg.current = current
        msg.percentage = percentage
        msg.design_capacity = 12.0                                      # 4 x 3000 mAh
        
        if current > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING 
        if current < 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if percentage == 1.0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL

        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.battery_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Init
    try:
        ina219 = INA219(addr=0x42)
    except OSError as err:
        rclpy.logging.get_logger('root').fatal("{}, init fail.".format(err))
        exit(1)
    node = MonitorBattery(ina219)

    # Spin
    rclpy.spin(node)


if __name__ == '__main__':
    main()
