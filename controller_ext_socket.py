import datetime
import json
from time import sleep

import serial


class DMXSocket:
    ser: serial.Serial

    def __init__(self):
        self.ser = serial.Serial('COM3', baudrate=9_600)
        print(self.ser.name)  # check which port was really used

    def terminate_connection(self):
        self.ser.close()  # close port

    def send_json(self, payload):

        # payload = dict(motor_on=payload['motor_on'])
        json_str = json.dumps(payload).replace(': ', ':').replace(', ', ',') + '\n'
        bytes_str = json_str.encode('ascii')

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

        # print(f"{controller_ext_msg=}")
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