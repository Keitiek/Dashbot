import average_slope_intercept
import pixel_points

# Function for detecting the right and left lane lines
def lane_lines1(image, lines):
    
    if lines is None or len(lines) == 0:
        return None, None
    
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = int(y1 * 0.45)
    left_line = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)
    return left_line, right_line