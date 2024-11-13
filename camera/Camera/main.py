import cv2
from object_detection import detect_objects
from lane_detection import lane_detection_process
from utils import calculate_forward_line_angle, determine_turn, get_lane_midpoint
import numpy as np

# Initialize camera
camera = cv2.VideoCapture(4)  # Camera index
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = camera.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Object detection
    detect_objects(frame)

    # Lane detection and processing
    lane_line_coords = lane_detection_process(frame)

    # Calculate forward line angle
    forward_angle = calculate_forward_line_angle(frame)

    # Midpoint and turn detection
    image_center_x = frame.shape[1] // 2
    midpoint = get_lane_midpoint(lane_line_coords[0], lane_line_coords[1], frame.shape[1])
    turn_direction = determine_turn(midpoint, image_center_x)

    # Draw midpoint if it exists
    if midpoint is not None:
        # Draw a circle at the midpoint
        cv2.circle(frame, (midpoint, frame.shape[0] - 30), 10, (255, 0, 0), -1)  # Red circle near bottom of the frame
        # Display turn direction text
        cv2.putText(frame, f"Turn: {turn_direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # Display detected information
    cv2.imshow('Lane and Object Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()
