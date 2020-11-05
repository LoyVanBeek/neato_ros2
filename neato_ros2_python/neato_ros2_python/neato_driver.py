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
import contextlib
import logging
import time

import serial

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

        self.base_width = 0.255
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

    def clear_input(self):
        while True:
            try:
                self.read_line()
            except Exception:
                logging.exception('Could not read another line')
                break

    def write_command(self, command: str, retries=100):
        if retries:
            logging.debug("Commanding '{}\'".format(command))
            self._port.write('{}\n'.format(command).encode('ascii'))

            echo_raw = self._port.readline()
            logging.debug("echo '{}'".format(echo_raw))
            echo = echo_raw.decode('ascii').strip()
            if command in echo:
                logging.debug(
                    "Serial port synced: written '{}' & got '{}'".format(
                        command, echo))
                logging.debug('Command written')
                return True
            elif 'Unknown Cmd' in echo:
                logging.debug("Error: unknown command '{}'".format(echo))
                return self.write_command(command, retries=retries - 1)
            elif 'Ambiguous Cmd' in echo:
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
            'setldsrotation {}'.format(
                'on' if on else 'off'))

    def set_motors(self, left_dist: int, right_dist: int, speed: int):
        assert self.write_command('setmotor {left} {right} {speed}'
                                  .format(left=int(left_dist),
                                          right=int(right_dist),
                                          speed=int(speed)))

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

    def start_laser_scan(self):
        """
        Start getting a laserscan
        """
        self._port.flushInput()
        assert self.write_command('getldsscan')

    def get_laser_scan(self):
        """
        Get the laser scan ranges in mm and the rotation speed (in RPM) of the laser scanner.

        :return: List of distances and rotation speed
        """
        self.start_laser_scan()
        return self.finish_laser_scan()

    def  finish_laser_scan(self):
        """
        Get the laser scan ranges in mm and the rotation speed (in RPM) of the laser scanner.

        :return: List of distances and rotation speed
        """
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


def main(args=None):
    robot = NeatoRobot(port='/dev/ttyACM0')

    with robot.operational():
        time.sleep(1)

        first = True

        while True:
            if not first:
                laser_ranges, _ = robot.get_laser_scan()
                print(laser_ranges)
            first = False
            motor_state = robot.get_motors()
            print(motor_state)
            robot.start_laser_scan()
            time.sleep(0.1)


if __name__ == '__main__':
    main()
