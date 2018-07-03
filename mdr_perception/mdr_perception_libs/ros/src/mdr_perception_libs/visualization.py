from enum import Enum
import numpy as np
import cv2


class BoundingBoxKey(Enum):
    LABEL = 'label'
    COLOR = 'color'
    X_MIN = 'x_min'
    X_MAX = 'x_max'
    Y_MIN = 'y_min'
    Y_MAX = 'y_max'


def visualize_boxes(image, boxes, line_width=2, font_scale=0.3, copy=True):
    if copy:
        drawn_image = image.copy()
    else:
        drawn_image = image
    default_color = (0, 0, 255)     # default color is blue
    for index, box in enumerate(boxes):
        color = default_color
        if BoundingBoxKey.COLOR in box and box[BoundingBoxKey.COLOR] is not None:
            color = box[BoundingBoxKey.COLOR]

        # end points
        point_top_left = (int(box[BoundingBoxKey.X_MIN]), int(box[BoundingBoxKey.Y_MIN]))
        point_bottom_right = (int(box[BoundingBoxKey.X_MAX]), int(box[BoundingBoxKey.Y_MAX]))

        # box
        cv2.rectangle(drawn_image, point_top_left, point_bottom_right, color, line_width)

        # label background
        text_size, baseline = cv2.getTextSize(box[BoundingBoxKey.LABEL], cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)
        text_top_left = (point_top_left[0], point_top_left[1] - text_size[1])
        text_bottom_right = (text_top_left[0] + text_size[0], point_top_left[1] + baseline)
        cv2.rectangle(drawn_image, text_top_left, text_bottom_right, color, cv2.FILLED)

        # label
        cv2.putText(drawn_image, box[BoundingBoxKey.LABEL], point_top_left, cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=font_scale, thickness=1, color=(255, 255, 255), lineType=cv2.LINE_AA)

    return drawn_image


def bgr_dict_from_classes(classes):
    """
    get dictionary of BGR colors span over HSV hue range from a list of classes

    :param classes: list of colors
    :return: dictionary {key=class name, value=color}
    """
    class_num = len(classes)
    np.random.seed(1234)
    np.random.shuffle(classes)

    hue = (np.linspace(0, 1, class_num) * 255).astype(np.uint8).reshape(1, class_num, 1)
    sat = np.ones((1, class_num, 1), dtype=np.uint8) * 127      # saturation 50%
    bright = np.ones((1, class_num, 1), dtype=np.uint8) * 127   # value/brightness 50%

    hsv = np.concatenate((hue, sat, bright), axis=2)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(int)

    return {class_name: tuple(bgr[0][index]) for index, class_name in enumerate(classes)}
