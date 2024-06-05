from controller_ext_socket import DMXSocket
from file_utills import get_json_from_file_if_exists, PIXEL_DEGREES_MAPPER_FILE_PATH
from state_machine import SauronEyeTowerStateMachine
from thermal_camera import ThermalEye

if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeTowerStateMachine(
        is_manual=False,
        socket=dmx_socket,
        thermal_eye=thermal_eye,
    )

    use_auto_scale_file = False
    try:
        if use_auto_scale_file:
            mapper_dict = get_json_from_file_if_exists(PIXEL_DEGREES_MAPPER_FILE_PATH)
            sauron.auto_coordinate(mapper_dict)

        sauron.do_evil()
    finally:
        dmx_socket.terminate_connection()
        thermal_eye.close_eye()
