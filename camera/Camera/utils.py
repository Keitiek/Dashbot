import numpy as np
import math
import cv2

# Function to calculate the average lanes from history
def average_lanes(lane_history):
    left_lines = [lane[0] for lane in lane_history if lane[0] is not None]
    right_lines = [lane[1] for lane in lane_history if lane[1] is not None]
    
    left_avg = np.mean(left_lines, axis=0) if len(left_lines) > 0 else None
    right_avg = np.mean(right_lines, axis=0) if len(right_lines) > 0 else None
    return left_avg, right_avg

# Function for calculating lane lines
def average_slope_intercept(lines):
    if lines is None or len(lines) == 0:
        return None, None
    
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))

    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane

# Function for getting pixel coordinates
def pixel_points(y1, y2, line):
    
    if line is None:
        return None
    
    slope, intercept = line
    
    if abs(slope) < 1e-6:
        return None
    
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return ((x1, int(y1)), (x2, int(y2)))

# Function for determining turn direction
def determine_turn(midpoint, image_center_x):
    if midpoint is None:
        return None
    deviation = midpoint - image_center_x
    if deviation > 45:
        return 'right'
    elif deviation < -45:
        return 'left'
    else:
        return 'straight'
    
# Function for defining a region of interest
def forward_region_of_interest(image):
    height, width = image.shape[:2]
    
    roi_height = height // 4
    roi_width = width // 4
    x1 = width // 2 - roi_width // 2
    x2 = width // 2 + roi_width // 2
    y1 = height - roi_height
    y2 = height
    
    return image[y1:y2, x1:x2]

# Function for calculateing the lane line angle within forward_region_of_interest
def calculate_forward_line_angle(image):
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

# Function for calculating midpoint between lanes or approximates 
def get_lane_midpoint(left_line, right_line, image_width):
    #print(f'Image_width: {image_width}')
    if left_line is not None and right_line is not None:
        left_x_bottom = left_line[0][0]
        right_x_bottom = right_line[0][0]
        midpoint = (left_x_bottom + right_x_bottom) // 2
        print(f"Left Line X: {left_x_bottom}, Right Line X: {right_x_bottom}")

    elif left_line is not None:
        midpoint = left_line[0][0] + (image_width // 4)
        print(f"Only left line detected. Left Line X: {left_line[0][0]}")
        
    elif right_line is not None:
        midpoint = right_line[0][0] - (image_width // 4)
        print(f"Only right line detected. Right Line X: {right_line[0][0]}")
    
    else:
        return None
    
    if 0 <= midpoint <= image_width:
        print(f'Midpoint:{midpoint}')
        return midpoint #Midpoint ei ole korralikult kalkuleeritud
    
    else:
        print(f'Midpoint out of bounds: {midpoint}')
        return None

def draw_lane_lines(image, lines, color=[255, 255, 0], thickness=12):
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            pt1, pt2 = line
            if isinstance(pt1, tuple) and isinstance(pt2, tuple):
                pt1 = tuple(map(int, pt1))
                pt2 = tuple(map(int, pt2))
                cv2.line(line_image, pt1, pt2, color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)
