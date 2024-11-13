import numpy as np

# Function for geting the average lane postion useing frames from lane_history
def average_lanes1(lane_history):
    left_lines = [lane[0] for lane in lane_history if lane[0] is not None]
    right_lines = [lane[1] for lane in lane_history if lane[1] is not None]
    
    left_avg = np.mean(left_lines, axis=0) if len(left_lines) > 0 else None
    right_avg = np.mean(right_lines, axis=0) if len(right_lines) > 0 else None
    return left_avg, right_avg