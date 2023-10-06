from dataclasses import dataclass

import numpy as np
import cv2


# Load YOLO model
net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')

# Load COCO class names
with open('coco.names', 'r') as f:
    classes = f.read().strip().split('\n')

# generate different colors for different classes
COLORS = np.random.uniform(0, 255, size=(len(classes), 3))


scale = 0.00392
def get_output_layers(net):
    layer_names = net.getLayerNames()
    output_layers = tuple(layer_names[i - 1] for i in net.getUnconnectedOutLayers())
    return output_layers


# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = str(classes[class_id])

    color = COLORS[class_id]

    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)

    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def mark_people(frame):
    height, width = frame.shape

    # create input blob
    blob = cv2.dnn.blobFromImage(frame, scale, (width, height),  swapRB=True, crop=False)

    # set input blob for the network
    net.setInput(blob)

    # run inference through the network
    # and gather predictions from output layers
    shapes = get_output_layers(net)
    outs = net.forward()

    # initialization
    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    # for each detetion from each output layer
    # get the confidence, class id, bounding box params
    # and ignore weak detections (confidence < 0.5)
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    # apply non-max suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    # go through the detections remaining
    # after nms and draw bounding box
    for i in indices:
        i = i[0]
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]

        draw_bounding_box(image, class_ids[i], confidences[i], round(x), round(y), round(x + w), round(y + h))

    return frame


@dataclass
class ThermalEye:
    cap: cv2.VideoCapture

    def search_ring_bearer(self):
        # captures a single frame
        ret, frame = self.cap.read()
        gray_scale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        target_x, target_y = mark_people(gray_scale_frame)
        height, width = frame.shape

        center_x = width / 2
        center_y = height / 2

        delta_x, delta_y = 0, 0
        if target_x < center_x:
            delta_x += 1
        elif target_x > center_x:
            delta_x -= 1

        if target_y < center_y:
            delta_y += 1
        elif target_y > center_y:
            delta_y -= 1

        cv2.imshow("frame", frame)

        return delta_x, delta_y


    def close_eye(self):
        self.cap.release()
        cv2.destroyAllWindows()


def get_thermal_eye_instance():
    cap = cv2.VideoCapture('thermal_video_test.mp4')
    return ThermalEye(
        cap=cap
    )


if __name__ == "__main__":

    thermal_eye = get_thermal_eye_instance()
    thermal_eye.search_ring_bearer()