import cv2
from ultralytics import YOLO
import numpy as np
import math
from lane_lines import lane_lines1
from calculate_forward_line_angle import calculate_forward_line_angle1
from average_lanes import average_lanes1
from draw_lane_lines import draw_lane_lines1
from get_lane_midpoint import get_lane_midpoint1
from determine_turn import determine_turn1

# Load the YOLO model (YOLOv8n is used in this example)
model = YOLO('yolov8n.pt')

# Get the class names from the model
class_names = model.names  # List of class names, indexed by class ID

# Camera parameters (corrected) #Come back to later
FOCAL_LENGTH = 48  # Focal length in mm (typical for See3CAM_CU20)
SENSOR_HEIGHT_MM = 368  # Correct sensor height for 1/3" sensor in mm
IMAGE_HEIGHT_PX = 48  # Image height in pixels for 640x480 resolution

# Open the camera
camera = cv2.VideoCapture(4)  # Adjust camera index if necessary
width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Camera Resolution: {int(width)}x{int(height)}")

# Set camera resolution
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

lane_history = []  # Buffer to store lane line coordinates over the last N frames
N = 5  # Number of frames to average over

road_not_detected_counter = 20  # Counter for frames where road is not detected
MAX_FRAMES_WITHOUT_LANES = 20  # Number of frames to consider before triggering "no road"

# Main loop for video stream processing
while True:
    ret, frame = camera.read()
    
    if not ret:
        print("Error: Could not read frame.")
        break

    # Use YOLO model to detect objects
    results = model(frame)

    # Start y-coordinate for the first overlay text
    start_y = 30  # Initial vertical position for displaying the text column
    line_height = 30  # Distance between each line of text
    
    OBJECT_CONFIDENCE_THRESHOLD = 0.4

    # Filter for specific classes (ID 0 for 'person', ID 7 for 'truck', ID 66 for 'traffic cone' etc.)
    detections_of_interest = []
    for result in results:
        for detection in result.boxes:
            if int(detection.cls) in range(255) and detection.conf.item() >= OBJECT_CONFIDENCE_THRESHOLD:  # Filter for classes of interest
                detections_of_interest.append(detection)

    # Annotate the frame with the detections
    for idx, detection in enumerate(detections_of_interest):
        # Get the bounding box coordinates
        x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Bounding box coordinates

        # Get the class name dynamically based on detection.cls
        class_id = int(detection.cls)
        class_name = class_names[class_id]

        # Get the confidence score for the detection
        confidence = detection.conf.item()

        # Calculate the height of the bounding box in pixels
        object_height_in_image_px = y2 - y1

        # Safeguard: Ensure object_height_in_image_px is not zero or too small
        if object_height_in_image_px <= 0:
            print(f"Bounding box height too small: {object_height_in_image_px}px. Skipping...")
            continue

        # Estimate distance using the pinhole camera model formula
        distance = (FOCAL_LENGTH * SENSOR_HEIGHT_MM) / (object_height_in_image_px * IMAGE_HEIGHT_PX)

        # Known real-world height of these objects
        KNOWN_PERSON_HEIGHT = 1.7  # Average height of a person in meters
        KNOWN_TRUCK_HEIGHT = 3.5  # Average height of a truck in meters
        KNOWN_TRAFFIC_CONE_HEIGHT = 0.45 # Minimal height of a traffic cone for 30mph-50mph in meters

        # Dynamically estimate the real height of the object based on detection
        if class_name == 'person':
            real_height = KNOWN_PERSON_HEIGHT
        elif class_name == 'truck':
            real_height = KNOWN_TRUCK_HEIGHT
        elif class_name == 'traffic cone':
            real_height = KNOWN_TRAFFIC_CONE_HEIGHT
        else:
            real_height = (object_height_in_image_px * distance) / FOCAL_LENGTH

        # Create a label with distance and real height
        overlay_text = f'{class_name} {confidence:.2f} Distance: {distance:.2f}m Height: {real_height:.2f}m'

        # Draw the bounding box on the frame
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the overlay text in a column, separated by 'line_height' for each object
        text_y = start_y + idx * line_height
        cv2.putText(frame, overlay_text, (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        
        # Begining of the stop if object infront of robot part of the code
        if 0.8 <= distance <= 1.0:
            if class_name == 'person':
                cv2.putText(frame, "STOP!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            else:
                cv2.putText(frame, "TURN!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
        elif 0.6 <= distance <= 0.8 or class_name == 'traffic cone':
            cv2.putText(frame, "REVERSE!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)


    # --- Line Detection Process ---
    # Convert the frame to HSV color space for white color segmentation
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for white color in HSV space
    lower_white = np.array([0, 0, 200])  # Lower HSV values for white
    upper_white = np.array([25, 0, 255])  # Upper HSV values for white
    
    # Create a mask that isolates white regions
    white_mask = cv2.inRange(hsv_frame, lower_white, upper_white)
    
    # Apply the mask to the frame to extract only white regions
    white_frame = cv2.bitwise_and(frame, frame, mask=white_mask)
    
    # Convert the masked white frame to grayscale
    gray_white_frame = cv2.cvtColor(white_frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection on the white regions
    edges = cv2.Canny(gray_white_frame, 100, 200, apertureSize=3)

    # Define region of interest mask (optional improvement)
    height, width = frame.shape[:2]
    mask = np.zeros_like(gray_white_frame)
    polygon = np.array([[
        (0, height), 
        (width, height), 
        (width, height // 2), 
        (0, height // 2)
    ]])
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Use masked_edges instead of edges in HoughLinesP
    lines = cv2.HoughLinesP(masked_edges, 5, np.pi / 180, threshold=100, minLineLength=40, maxLineGap=40)

    lane_line_coords = lane_lines1(frame, lines)
    
    forward_angle = calculate_forward_line_angle1(frame)
    
    if forward_angle is not None:
        cv2.putText(frame, f'Forward Angle: {forward_angle:.2f} degrees', (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        print(f"Forward Line Angle: {forward_angle:.2f} degrees")
    else:
        cv2.putText(frame, "No Forward Line Detected", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Inside the main loop after detecting lane lines:
    lane_line_coords = lane_lines1(frame, lines)
    lane_history.append(lane_line_coords)

    if len(lane_history) > N:
        lane_history.pop(0)

    smoothed_lane_coords = average_lanes1(lane_history)
    frame = draw_lane_lines1(frame, smoothed_lane_coords)
    
    image_center_x = frame.shape[1] // 2
    midpoint = get_lane_midpoint1(smoothed_lane_coords[0], smoothed_lane_coords[1], frame.shape[1])
    turn_direction = determine_turn1(midpoint, image_center_x)
    
    if midpoint is not None:
        cv2.circle(frame, (int(midpoint), frame.shape[0] - 10), 10, (255, 0, 0), -1)
        
    if turn_direction == 'left':
        cv2.putText(frame, "Future Movement: Turning Left", (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.arrowedLine(frame, (frame.shape[1]//2, frame.shape[0] - 100), (frame.shape[1]//2 - 100, frame.shape[0] - 150), (0, 255, 255), 5)
    elif turn_direction == 'right':
        cv2.putText(frame, "Future Movement: Turning Right", (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.arrowedLine(frame, (frame.shape[1]//2, frame.shape[0] - 100), (frame.shape[1]//2 + 100, frame.shape[0] - 150), (0, 255, 255), 5)
    elif turn_direction == 'straight':
        cv2.putText(frame, "Future Movement: Going Straight", (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.arrowedLine(frame, (frame.shape[1]//2, frame.shape[0] - 100), (frame.shape[1]//2, frame.shape[0] - 200), (0, 255, 255), 5)
    else:
        cv2.putText(frame, "No Lane Detected", (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    if lane_line_coords[0] is None and lane_line_coords[1] is None:
        road_not_detected_counter += 1
        
    else:
        road_not_detected_counter = 0
        
    if road_not_detected_counter >= MAX_FRAMES_WITHOUT_LANES:
        cv2.putText(frame, "Warning: Road not detected!", (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
    else:
        road_not_detected_counter = 0
    
    frame = draw_lane_lines1(frame, lane_line_coords)
    # Draw lane lines
    if lines is not None:
        lane_line_coords = lane_lines1(frame, lines)
        frame = draw_lane_lines1(frame, lane_line_coords)

    # Show the frame with the detected objects and lane lines
    cv2.imshow('People, Truck Detection, and Lane Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
camera.release()
cv2.destroyAllWindows()
