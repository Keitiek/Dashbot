# Function for determening if he robot should turn left, right or go straight
def determine_turn1(midpoint, image_center_x):
    
    if midpoint is None:
        #print(f'image_center_x: {image_center_x}')
        return None
    
    deviation = midpoint - image_center_x
    print(f"Midpoint: {midpoint}, Image Center: {image_center_x}, Deviation: {deviation}")
    
    if deviation > 45:
        return 'right'
    
    elif deviation < -45:
        return 'left'
    
    else:
        return 'straight'