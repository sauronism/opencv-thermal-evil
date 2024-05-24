import os


def stringify_vector_dict(data):
    return {
        str(key): value
        for key, value in data.items()
    }


def unstringify_vector_dict(data):
    return {
        tuple(key): value
        for key, value in data.items()
    }


def save_json_file(file_path, data):
    with open(file_path, 'wb') as f:
        dict_str = str(data).encode('utf-8')
        f.write(dict_str)


def get_json_from_file_if_exists(file_path):
    if not os.path.isfile(file_path):
        return {}

    try:
        with open(file_path, 'rb') as f:
            dict_str = f.read().decode('utf-8')
            pixel_degrees_mapper = eval(dict_str)
    except Exception as e:
        print(e)
        pixel_degrees_mapper = {}
    return pixel_degrees_mapper
