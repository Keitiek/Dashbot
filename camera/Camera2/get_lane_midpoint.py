# Function for calculating midpoint between lanes or approximates 
def get_lane_midpoint1(left_line, right_line, image_width):
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