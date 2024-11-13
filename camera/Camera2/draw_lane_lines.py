import numpy as np
import cv2

# Function for drawing lane lines
def draw_lane_lines1(image, lines, color=[255, 255, 0], thickness=12):
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            pt1, pt2 = line
            if isinstance(pt1, tuple) and isinstance(pt2, tuple):
                pt1 = tuple(map(int, pt1))
                pt2 = tuple(map(int, pt2))
                cv2.line(line_image, pt1, pt2, color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)
