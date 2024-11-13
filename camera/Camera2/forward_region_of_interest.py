# Function for defining a region of interest
def forward_region_of_interest1(image):
    height, width = image.shape[:2]
    
    roi_height = height // 4
    roi_width = width // 4
    x1 = width // 2 - roi_width // 2
    x2 = width // 2 + roi_width // 2
    y1 = height - roi_height
    y2 = height
    
    return image[y1:y2, x1:x2]