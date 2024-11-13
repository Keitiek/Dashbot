import math

# Function for calculateing the angle of forward lines
def calculate_line_angle1(line):
    if line is None:
        return None
    
    (x1, y1), (x2, y2) = line
    
    delta_y = y2 - y1
    delta_x = x2 - x1
    
    if delta_x == 0:
        return 90
    
    angle_radians = math.atan2(delta_y, delta_x)
    
    return angle_radians