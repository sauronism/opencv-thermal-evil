import datetime
import glob
import json
import sys
from time import sleep

import serial

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result



class DMXSocket:
    ser: serial.Serial

    def __init__(self):
        print(serial_ports())

        self.ser = serial.Serial('/dev/tty.usbmodem21301', baudrate=256_000)
        print(self.ser.name)  # check which port was really used

    def terminate_connection(self):
        self.ser.close()  # close port

    def send_json(self, payload: dict):

        # payload = dict(motor_on=payload['motor_on'])
        json_str = json.dumps(payload).replace(': ', ':').replace(', ', ',')

        bytes_str = json_str.encode('utf-8')

        print(bytes_str)

        self.ser.write(bytes_str)  # write a string
        # self.ser.flush()

        return self.read_controller_ext_msg()

    def read_controller_ext_msg(self):
        # try:
        #     controller_ext_msg = self.ser.readline().decode("utf-8")
        #     print(controller_ext_msg)
        # except Exception as e:
        #     print(e)
        #     pass
        sleep(0.1)

        controller_ext_msg = ''
        while True:
            bytes_to_read = self.ser.inWaiting()

            if bytes_to_read:
                received_bytes = self.ser.read(bytes_to_read).decode()
                print(received_bytes)
                controller_ext_msg += received_bytes

            if bytes_to_read == 0:
                break

        print(f"{controller_ext_msg=}")
        # while True:
        #     print(self.ser.read().decode())

        return controller_ext_msg


if __name__ == "__main__":
    socket = DMXSocket()
    try:
        payload = dict(
            motor_on=True
        )
        socket.send_json(payload)
    finally:
        socket.terminate_connection()


    # import serial.tools.list_ports as port_list
    #
    # ports = list(port_list.comports())
    # for p in ports:
    #     print(p)