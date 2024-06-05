import datetime
import glob
import json
import sys
from time import sleep
from typing import Optional

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

    instruction_payload: Optional[dict] = None

    def __init__(self):
        print(serial_ports())
        try:
            self.ser = serial.Serial('COM5', baudrate=256_000)
            print(self.ser.name)  # check which port was really used
        except Exception as e:
            print('no connection to dmx - cam only mode. {e}')  # check which port was really used
            self.ser = None

    def terminate_connection(self):
        if self.ser is None:
            print('no connection to dmx - cam only mode.')
            return

        self.ser.close()  # close port

    def send_json(self, instruction_payload: Optional[dict] = None, print_return_payload=True):
        if self.ser is None:
            print('no connection to dmx - cam only mode.')
            return

        instruction_payload = instruction_payload or self.instruction_payload
        if instruction_payload is None:
            return

        json_str = json.dumps(instruction_payload).replace(': ', ':').replace(', ', ',')
        bytes_str = json_str.encode('utf-8')
        if print_return_payload:
            print(bytes_str)

        self.ser.write(bytes_str)  # write a string
        # self.ser.flush()

        return self.read_controller_ext_msg(print_return_payload=print_return_payload)

    def read_controller_ext_msg(self, print_return_payload=True):
        if self.ser is None:
            print('no connection to dmx - cam only mode.')
            return

        controller_ext_msg = ''

        # Arduino response time
        # sleep(0.01)

        while True:
            bytes_to_read = self.ser.inWaiting()

            if bytes_to_read:
                received_bytes = self.ser.read(bytes_to_read).decode()
                if print_return_payload:
                    print(received_bytes)
                controller_ext_msg += received_bytes

            if bytes_to_read == 0:
                break
        if print_return_payload:
            print(f"{controller_ext_msg=}")

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