import math
import cv2
import forward_region_of_interest
import numpy as np

# Function for calculateing the lane line angle within forward_region_of_interest
def calculate_forward_line_angle1(image):
    forward_roi = forward_region_of_interest(image)
    
    gray = cv2.cvtColor(forward_roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=30, maxLineGap=10)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                angle_radians = math.atan2((y2 - y1), (x2 - x1))
                angle_degrees = math.degrees(angle_radians)
                
                cv2.line(forward_roi, (x1, y1), (x2, y2), ( 0, 0, 255), 2)
                
                return angle_degrees
    
    return None