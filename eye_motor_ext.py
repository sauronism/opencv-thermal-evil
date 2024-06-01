import requests

ESP_ADDRESS = '192.168.4.1'


def send_motor_instruction(display_on: bool, eye_azimuth: int = 0):
    data = {
        'display_on': display_on,
        'eye_azimuth': eye_azimuth,
        'display_custom_text': False,
        'custom_text_data': "hello world"
    }

    # Step 3: Make the POST request
    response = requests.post(f"http://{ESP_ADDRESS}/json_client", json=data)

    # Print the response text
    print(response.text)


if __name__ == '__main__':
    send_motor_instruction(display_on=True)
    # send_motor_instruction(display_on=False)
