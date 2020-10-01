from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import serial
import contextlib
import rclpy
from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from tf2_ros.transform_broadcaster import TransformBroadcaster
import time
import logging
import numpy as np

logging.basicConfig(level=logging.INFO)


MOTOR_STATUS_FIELDS = [
    'brush_rpm',
    'brush_ma',
    'vacuum_rpm',
    'vacuum_ma',
    'left_wheel_rpm',
    'left_wheel_load',
    'left_wheel_position_in_mm',
    'left_wheel_speed',
    'right_wheel_rpm',
    'right_wheel_load',
    'right_wheel_position_in_mm',
    'right_wheel_speed',
    'side_brush_ma']


class NeatoRobot(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self._port = serial.Serial(port, baudrate, timeout=0.1)
        self._motor_state = None

        self._laser_line_count = 360

        self.base_width = 0.245
        self.max_speed = 0.300  # Meters

    def __del__(self):
        self._port.close()

    @contextlib.contextmanager
    def operational(self):
        self._port.flushInput()
        self._port.flushOutput()

        self.set_testmode(True)
        self.set_ldsrotation(True)

        yield

        self.set_ldsrotation(False)
        self.set_testmode(False)

        self._port.flushOutput()
        self._port.flushInput()

    def write_command(self, command: str, retries=100):
        if retries:
            logging.debug('Commanding \'{}\''.format(command))
            self._port.write("{}\n".format(command).encode('ascii'))

            echo_raw = self._port.readline()
            logging.debug("echo '{}'".format(echo_raw))
            echo = echo_raw.decode('ascii').strip()
            if command in echo:
                logging.debug(
                    "Serial port synced: written '{}' & got '{}'".format(
                        command, echo))
                logging.debug('Command written')
                return True
            elif "Unknown Cmd" in echo:
                logging.debug("Error: unknown command '{}'".format(echo))
                return self.write_command(command, retries=retries - 1)
            elif "Ambiguous Cmd" in echo:
                logging.debug("Error: ambiguous command '{}'".format(echo))
                return self.write_command(command, retries=retries - 1)
            else:
                logging.debug(
                    "Serial port not yet in sync, expected '{}', got '{}'".format(
                        command, echo))
                return self.write_command(command, retries=retries - 1)
        else:
            return False

    def read_line(self):
        raw = self._port.readline()
        _ascii = raw.decode('ascii')
        stripped = _ascii.strip()
        logging.debug("'{}' -> '{}' -> '{}'".format(raw, _ascii, stripped))
        return stripped

    def set_testmode(self, on: bool):
        assert self.write_command('testmode {}'.format('on' if on else 'off'))

    def set_ldsrotation(self, on: bool):
        assert self.write_command(
            'etldsrotation {}'.format(
                'on' if on else 'off'))

    def set_motors(self, left_dist: int, right_dist: int, speed: int):
        assert self.write_command('setmotor {l} {r} {s}'
                                  .format(l=int(left_dist),
                                          r=int(right_dist),
                                          s=int(speed)))

    def get_motors(self):
        # self._port.flushInput()
        assert self.write_command('getmotors')
        logging.debug('Getting header...: ')
        header = ''
        for i in range(100):
            if 'Parameter' not in header:
                header = self.read_line()
                logging.debug(header)
            else:
                break
        else:
            logging.debug('Did not get header in time')
            raise TimeoutError('Did not get header in time')
        logging.debug('Got complete header, now reading  actual motor state')
        status = {}

        for _ in MOTOR_STATUS_FIELDS:
            # self.logger.debug('Getting line...: ')
            line = self.read_line()
            # self.logger.debug(line)
            parts = line.split(',')
            status[parts[0]] = int(parts[1])
        self._motor_state = status
        return status

    def get_laser_scan(self):
        """
        Get the laser scan ranges in mm and the rotation speed (in RPM) of the laser scanner.

        :return: List of distances and rotation speed
        """
        self._port.flushInput()
        assert self.write_command('getldsscan')

        _ = self._port.readline().decode('utf-8')  # Read header

        ranges = [0] * self._laser_line_count

        for expected_angle in range(self._laser_line_count):
            scanline = self.read_line()
            parts = scanline.split(',')
            angle = int(parts[0])
            distance = int(parts[1])
            ranges[angle] = distance  # Distance millimeters

        footer = self.read_line()

        lds_rpm = float(footer.split(',')[1])

        return ranges, lds_rpm


class NeatoNode(Node):
    def __init__(self, robot: NeatoRobot):
        super(NeatoNode, self).__init__('neato')

        self._robot = robot

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
        self._scan.angle_max = np.pi * 2
        self._scan.angle_increment = (
            self._scan.angle_max - self._scan.angle_min) / 360.0
        self._scan.range_min = 0.020
        self._scan.range_max = 5.0

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self._encoders = [0, 0]
        self._odom = Odometry(header=Header(frame_id='odom'),
                              child_frame_id='base_link')

        self._bl_tf = TransformStamped(header=Header(frame_id='odom'),
                                       child_frame_id='base_link')
        self._bl_tf.transform.translation.x = 0.0
        self._bl_tf.transform.translation.y = 0.0
        self._bl_tf.transform.translation.z = 0.0
        self._bl_tf.transform.rotation.w = 1.0
        self._bl_tf.transform.rotation.x = 0.0
        self._bl_tf.transform.rotation.y = 0.0
        self._bl_tf.transform.rotation.z = 0.0

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
        dt_secs = dt.nanoseconds / 1_000_000_000

        self.get_logger().debug('tick')
        motor_state = self._robot.get_motors()

        self.get_logger().debug('tack')
        self._robot.set_motors(**self.motor_commands)

        self.get_logger().debug('teck')
        laser_ranges, laser_rpm = self._robot.get_laser_scan()

        self.get_logger().debug('tuck')
        self._scan.ranges = list(np.array(laser_ranges) / 1000)

        self._scan.header.stamp = now.to_msg()
        self._scan_pub.publish(self._scan)

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


def main(args=None):
    rclpy.init(args=args)
    logging.info('Hi from neato_ros2_python.')

    robot = NeatoRobot(port='/dev/ttyACM0')

    with robot.operational():
        node = NeatoNode(robot)

        time.sleep(1)

        # rclpy.spin(node)
        logging.info('Robot operational, starting loop')
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
