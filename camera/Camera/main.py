import cv2
from object_detection import detect_objects
from lane_detection import lane_detection_process, draw_lane_lines
from utils import calculate_forward_line_angle, determine_turn, get_lane_midpoint
from motor_control import stop_motors, move_forward, turn_left, turn_right, reverse

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

    # Draw lane lines on the frame
    if lane_line_coords is not None:
        frame = draw_lane_lines(frame, lane_line_coords)  # Visualize the detected lane lines

    # Calculate forward line angle
    forward_angle = calculate_forward_line_angle(frame)

    # Midpoint and turn detection
    image_center_x = frame.shape[1] // 2
    if lane_line_coords is not None:
        midpoint = get_lane_midpoint(lane_line_coords[0], lane_line_coords[1], frame.shape[1])
    else:
        midpoint = None

    turn_direction = determine_turn(midpoint, image_center_x)

    # Motor control logic
    if turn_direction == 'left':
        turn_left()
    elif turn_direction == 'right':
        turn_right()
    elif turn_direction == 'straight':
        move_forward()
    else:
        stop_motors()

    # Display detected information
    if midpoint is not None:
        cv2.circle(frame, (midpoint, frame.shape[0] - 30), 10, (255, 0, 0), -1)  # Blue circle 
        cv2.putText(frame, f"Turn: {turn_direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow('Camera', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop motors and release resources
stop_motors()
camera.release()
cv2.destroyAllWindows()
