import serial
import rclpy
import time


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

    def __del__(self):
        self._port.close()

    def write_command(self, command: str):
        self._port.write("{}\n".format(command).encode('utf-8'))
        for i in range(100):
            echo = self._port.readline().decode('utf-8').strip()
            if command in echo:
                break
            else:
                print("Serial port not yet in sync")

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


def main():
    print('Hi from neato_ros2_python.')

    neato = NeatoRobot(port='/dev/ttyACM0')

    neato.set_testmode(True)
    neato.set_ldsrotation(True)
    time.sleep(1)
    print(neato.get_motors())
    print(neato.get_laser_scan())


if __name__ == '__main__':
    main()
