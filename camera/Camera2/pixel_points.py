# Function for getting pixel coordinates
def pixel_points1(y1, y2, line):
    
    if line is None:
        return None
    
    slope, intercept = line
    
    if abs(slope) < 1e-6:
        return None
    
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return ((x1, int(y1)), (x2, int(y2)))