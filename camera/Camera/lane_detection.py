import cv2
import numpy as np
from utils import average_lanes, draw_lane_lines, average_slope_intercept, pixel_points

def lane_detection_process(frame):
    # Convert to HSV and apply white color detection
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([25, 0, 255])
    white_mask = cv2.inRange(hsv_frame, lower_white, upper_white)
    white_frame = cv2.bitwise_and(frame, frame, mask=white_mask)
    gray_white_frame = cv2.cvtColor(white_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_white_frame, 100, 200)

    # Region of interest
    height, width = frame.shape[:2]
    mask = np.zeros_like(gray_white_frame)
    polygon = np.array([[(0, height), (width, height), (width, height // 2), (0, height // 2)]])
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Hough line detection
    lines = cv2.HoughLinesP(masked_edges, 5, np.pi / 180, threshold=100, minLineLength=40, maxLineGap=40)
    lane_line_coords = lane_lines(frame, lines)

    return lane_line_coords

def lane_lines(image, lines):
    if lines is None or len(lines) == 0:
        return None, None
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = int(y1 * 0.45)
    left_line = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)
    return left_line, right_line
