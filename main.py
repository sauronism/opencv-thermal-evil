from controller_ext_socket import DMXSocket
from state_machine import SauronEyeTowerStateMachine
from thermal_camera import ThermalEye

if __name__ == '__main__':
    thermal_eye = ThermalEye(0)
    dmx_socket = DMXSocket()

    sauron = SauronEyeTowerStateMachine(
        is_manual=True,
        socket=dmx_socket,
        thermal_eye=thermal_eye,

        use_auto_scale_file=True,
    )
    try:
        sauron.do_evil()
    finally:
        dmx_socket.terminate_connection()
        thermal_eye.close_eye()
