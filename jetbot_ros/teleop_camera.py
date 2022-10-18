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
import string
import scipy.spatial.transform
# ROS
import rclpy
import rclpy.node
import rclpy.logging
import sensor_msgs.msg
# Pan-Tilt
from jetbot_ros.PCA9685 import PCA9685


PAN_MIN = 10
PAN_MAX = 170
PAN_OFFSET = 0

TILT_MIN = 0
TILT_MAX = 80
TILT_OFFSET = 10

class TeleopCamera(rclpy.node.Node):
 
    def __init__(self):
        super().__init__('teleop_camera_node')

        self.pan  = (PAN_MAX + PAN_MIN) / 2 + PAN_OFFSET
        self.tilt = (TILT_MAX + TILT_MIN) / 2 + TILT_OFFSET

        # Subscribed Topics
        self.imu_sub = self.create_subscription(sensor_msgs.msg.Imu, '/imu', self.imu_cb, 1)
        self.joy_sub = self.create_subscription(sensor_msgs.msg.Joy, '/joy', self.joy_cb, 1)


    def imu_cb(self, msg):
    
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        R = scipy.spatial.transform.Rotation.from_quat([qx, qy, qz, qw])

        euler = R.as_euler('xyz', degrees=True)
        roll  = euler[0]
        pitch = euler[1]
        yaw   = euler[2]

        self.get_logger().debug("rpy: {:.1f} {:.1f} {:.1f}".format(roll, pitch, yaw))

        self.pan  = (PAN_MAX + PAN_MIN) / 2 + PAN_OFFSET - yaw
        self.tilt = (TILT_MAX + TILT_MIN) / 2 + TILT_OFFSET - pitch

        self.pan = max(self.pan, PAN_MIN)
        self.pan = min(PAN_MAX, self.pan)

        self.tilt = max(self.tilt, TILT_MIN)
        self.tilt = min(TILT_MAX, self.tilt)

        self.get_logger().debug("pan, tilt: {:.1f} {:.1f}".format(self.pan, self.tilt))


    def joy_cb(self, msg):
    
        self.get_logger().debug("axes: {}".format(msg.axes))

        PAN_SCALE = 2
        TILT_SCALE = 2
        
        pan_inc  = PAN_SCALE * round(msg.axes[6])
        tilt_inc = TILT_SCALE * round(msg.axes[7])

        self.pan += pan_inc
        self.pan = max(self.pan, PAN_MIN)
        self.pan = min(PAN_MAX, self.pan)

        self.tilt += tilt_inc
        self.tilt = max(self.tilt, TILT_MIN)
        self.tilt = min(TILT_MAX, self.tilt)

        # recenter camera
        if msg.buttons[0]:
            self.pan  = (PAN_MAX + PAN_MIN) / 2 + PAN_OFFSET
            self.tilt = (TILT_MAX + TILT_MIN) / 2 + TILT_OFFSET

        self.get_logger().debug("pan, tilt: {:.1f} {:.1f}".format(self.pan, self.tilt))


def main(args=None):    
    rclpy.init(args=args)

    # Node
    node = TeleopCamera()

    # Init PCA9685
    try:
        pwm = PCA9685()
        pwm.setPWMFreq(50)
    except OSError as ex:
        node.get_logger().fatal("{}".format(ex))
        exit(1)

    # rate = node.create_rate(10)
    while rclpy.ok():

        # rate.sleep()
        
        pwm.setRotationAngle(1, node.pan)
        pwm.setRotationAngle(0, node.tilt)
        
        rclpy.spin_once(node)

    pwm.exit_PCA9685()

if __name__ == '__main__':
    main()
