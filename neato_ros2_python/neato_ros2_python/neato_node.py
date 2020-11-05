"""
Copyright 2020 Loy van Beek.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the 'Software'), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""
import time
from typing import List

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node, Parameter, ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster

from .neato_driver import NeatoRobot

lX, lY, lZ, rX, rY, rZ = 0, 1, 2, 3, 4, 5


class NeatoNode(Node):

    def __init__(self, robot: NeatoRobot):
        super(NeatoNode, self).__init__('neato')

        self._robot = robot
        self._expect_laser_scan = False

        self.declare_parameter('base_width', self._robot.base_width,
                               ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                   description='How far are the wheels apart?'))

        self._scan_pub = self.create_publisher(LaserScan, 'scan', 1)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._process_cmd_vel, 1)

        self.motor_commands = {'left_dist': 0,
                               'right_dist': 0,
                               'speed': 0}

        self.declare_parameter(
            'frame_id',
            'laser_link',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Frame ID for the laser'))
        scan_link = self.get_parameter_or('frame_id').value
        self._scan = LaserScan(header=Header(frame_id=scan_link))
        self._scan.angle_min = 0.0
        self._scan.angle_increment = (np.pi * 2) / 360.0
        self._scan.angle_max = (np.pi * 2) - self._scan.angle_increment
        self._scan.range_min = 0.020
        self._scan.range_max = 5.0

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self._encoders = [0, 0]
        self._odom = Odometry(header=Header(frame_id='odom'),
                              child_frame_id='base_footprint')

        self._odom.pose.covariance[lX*6 + lX] = 0.01
        self._odom.pose.covariance[lY*6 + lY] = 0.01
        self._odom.pose.covariance[lZ*6 + lZ] = 0
        self._odom.pose.covariance[rX*6 + rX] = 0
        self._odom.pose.covariance[rY*6 + rY] = 0
        self._odom.pose.covariance[rZ*6 + rZ] = 0.01

        self._odom.twist.covariance[lX*6 + lX] = 0.02
        # self._odom.twist.covariance[lY*6 + lY] = 0.0001
        self._odom.twist.covariance[lZ*6 + lZ] = 0
        # self._odom.twist.covariance[rX*6 + rX] = 0.1
        # self._odom.twist.covariance[rY*6 + rY] = 0.1
        self._odom.twist.covariance[rZ*6 + rZ] = 0.02

        self._bl_tf = TransformStamped(header=Header(frame_id='odom'),
                                       child_frame_id='base_footprint')
        self._bl_tf.transform.translation.x = 0.0
        self._bl_tf.transform.translation.y = 0.0
        self._bl_tf.transform.translation.z = 0.0
        self._bl_tf.transform.rotation.w = 1.0
        self._bl_tf.transform.rotation.x = 0.0
        self._bl_tf.transform.rotation.y = 0.0
        self._bl_tf.transform.rotation.z = 0.0

        self.get_logger().debug('Adding callback')
        self.add_on_set_parameters_callback(self._handle_parameters)

    def _process_cmd_vel(self, twist: Twist):
        self.get_logger().debug('twist: {}'.format(twist))

        x = twist.linear.x
        th = twist.angular.z * (self._robot.base_width / 2)
        k = max(abs(x - th), abs(x + th))

        self.get_logger().debug('x: {}, th: {}, k: {}'.format(x, th, k))

        # sending commands higher than max speed will fail
        if k > self._robot.max_speed:
            factor = self._robot.max_speed / k

            x *= factor
            th *= factor

            self.get_logger().debug(
                'Scaling velocities down by {}: x: {}, th: {}'.format(
                    factor, x, th))
        left, right = x - th, x + th

        speed = max(abs(left),
                    abs(right))
        self.get_logger().debug(
            'Motor commands: left: {}: right: {}, speed: {}'.format(
                left, right, speed))

        self.motor_commands = {'left_dist': int(left * 1000),
                               'right_dist': int(right * 1000),
                               'speed': int(speed * 1000)}

    def tick(self, previous_time):
        now = self.get_clock().now()
        dt = now - previous_time
        dt_secs = dt.nanoseconds / 1000000000

        self.get_logger().debug('tick')
        if self._expect_laser_scan:
            laser_ranges, laser_rpm = self._robot.finish_laser_scan()

            self.get_logger().debug('tuck')
            self._scan.ranges = list(np.array(laser_ranges) / 1000)

            self._scan.header.stamp = now.to_msg()
            self._scan_pub.publish(self._scan)

        self.get_logger().debug('tack')
        motor_state = self._robot.get_motors()

        self.get_logger().debug('teck')
        self._robot.set_motors(**self.motor_commands)

        self._robot.start_laser_scan()
        self._expect_laser_scan = True

        d_left = (motor_state['LeftWheel_PositionInMM'] -
                  self._encoders[0]) / 1000.0
        d_right = (
            motor_state['RightWheel_PositionInMM'] - self._encoders[1]) / 1000.0
        self._encoders = [motor_state['LeftWheel_PositionInMM'],
                          motor_state['RightWheel_PositionInMM']]

        dx = (d_left + d_right) / 2
        dth = (d_right - d_left) / self._robot.base_width

        x = np.cos(dth) * dx
        y = -np.sin(dth) * dx
        self.x += np.cos(self.th) * x - np.sin(self.th) * y
        self.y += np.sin(self.th) * x + np.cos(self.th) * y
        self.th += dth

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = np.sin(self.th / 2.0)
        quaternion.w = np.cos(self.th / 2.0)

        # Fill in odometry
        self._odom.header.stamp = now.to_msg()
        self._odom.pose.pose.position.x = self.x
        self._odom.pose.pose.position.y = self.y
        self._odom.pose.pose.position.z = 0.0
        self._odom.pose.pose.orientation = quaternion
        self._odom.twist.twist.linear.x = dx / dt_secs
        self._odom.twist.twist.angular.z = dth / dt_secs

        self._odom_pub.publish(self._odom)

        self._bl_tf.header.stamp = now.to_msg()
        self._bl_tf.transform.translation.x = self.x
        self._bl_tf.transform.translation.y = self.y
        self._bl_tf.transform.rotation = quaternion

        self._tf_broadcaster.sendTransform(self._bl_tf)

        self.get_logger().debug('tock')

    def _handle_parameters(self, parameters: List[Parameter]):
        for parameter in parameters:
            if parameter.name == 'base_width':
                self.get_logger().warn("Overriding robot's base_width: {}".format(parameter.value))
                self._robot.base_width = parameter.value

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    robot = NeatoRobot(port='/dev/ttyACM0')

    with robot.operational():
        node = NeatoNode(robot)

        time.sleep(1)

        # rclpy.spin(node)
        node.get_logger().info('Robot operational, starting loop')
        prev = node.get_clock().now()
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
                node.tick(prev)
                prev = node.get_clock().now()
            except KeyboardInterrupt:
                break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
