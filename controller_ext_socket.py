import json

import serial


class DMXSocket:
    ser: serial.Serial

    def __init__(self):
        self.ser = serial.Serial('COM3')
        print(self.ser.name)  # check which port was really used

    def terminate_connection(self):
        self.ser.close()  # close port

    def send_json(self, payload):
        json_str = bytes(json.dumps(payload), 'utf-8')
        self.ser.write(json_str)  # write a string

        cont_ext_msg = self.ser.read().decode()
        return cont_ext_msg


if __name__ == "__main__":
    try:
        socket = DMXSocket()

        payload = dict(
        motor_on=False
        )
        socket.send_json(payload)
    finally:
        socket.terminate_connection()


    # import serial.tools.list_ports as port_list
    #
    # ports = list(port_list.comports())
    # for p in ports:
    #     print(p)