import serial
import contextlib
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


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
        self._port = serial.Serial(port, baudrate)

        self._motor_state = None

        self._laser_line_count = 360

        self.base_width = 0.245
        self.max_speed = 0.300  # Meters

    def __del__(self):
        self._port.close()

    @contextlib.contextmanager
    def operational(self):
        self.set_testmode(True)
        self.set_ldsrotation(True)

        yield

        self.set_ldsrotation(False)
        self.set_testmode(False)


    def write_command(self, command: str):
        self._port.write("{}\n".format(command).encode('utf-8'))
        for i in range(100):
            echo = self._port.readline().decode('utf-8').strip()
            if command in echo:
                # print("Serial port synced")
                break
            else:
                # print("Serial port not yet in sync, expected '{}', got '{}'".format(command, echo))
                pass

    def read_line(self):
        return self._port.readline().decode('utf-8').strip()

    def set_testmode(self, on: bool):
        self.write_command("testmode {}".format('on' if on else 'off'))

    def set_ldsrotation(self, on: bool):
        self.write_command("setldsrotation {}".format('on' if on else 'off'))

    def set_motors(self, left_dist: int, right_dist: int, speed: int):
        self.write_command("setmotor {l} {r} {s}"
                           .format(l=int(left_dist),
                                   r=int(right_dist),
                                   s=int(speed)))

    def get_motors(self):
        self._port.flushInput()
        self.write_command("getmotors")
        header = self.read_line()
        status = {}

        for _ in MOTOR_STATUS_FIELDS:
            line = self.read_line()
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
        self.write_command("getldsscan")

        header = self._port.readline().decode('utf-8')

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

        self.create_publisher(LaserScan, 'scan', 1)
        self.create_publisher(Odometry, 'odom', 1)

        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self._process_cmd_vel, 10)

        # TODO: Do this on a timer
        # time.sleep(1)
        # print(robot.get_motors())
        # print(robot.get_laser_scan())

    def _process_cmd_vel(self, twist: Twist):
        self.get_logger().debug('twist: {}'.format(twist))

        x = twist.linear.x
        th = twist.angular.z * (self._robot.base_width/2)
        k = max(abs(x-th), abs(x+th))

        self.get_logger().debug('x: {}, th: {}, k: {}'.format(x, th, k))

        # sending commands higher than max speed will fail
        if k > self._robot.max_speed:
            factor = self._robot.max_speed/k

            x *= factor
            th *= factor

            self.get_logger().debug('Scaling velocities down by {}: x: {}, th: {}'.format(factor, x, th))
        left, right = x-th, x+th

        speed = max(abs(left),
                    abs(right))
        self.get_logger().debug('Motor commands: left: {}: right: {}, speed: {}'.format(left, right, speed))

        self._robot.set_motors(left_dist=int(left*1000),
                               right_dist=int(right*1000),
                               speed=int(speed*1000))


def main(args=None):
    rclpy.init(args=args)
    print('Hi from neato_ros2_python.')

    robot = NeatoRobot(port='/dev/ttyACM0')

    with robot.operational():
        node = NeatoNode(robot)

        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except KeyboardInterrupt:
                break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
