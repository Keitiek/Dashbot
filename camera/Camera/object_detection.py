import cv2
from ultralytics import YOLO
import math
from motor_control import stop_motors, reverse, turn_left, turn_right

# Load YOLO model
model = YOLO('yolov8n.pt')

# Constants for camera and known object heights
FOCAL_LENGTH_PX = 120 #120  # Focal length in mm (change as per your camera)
CAMERA_HEIGHT = 0.84 #0.25 Robotil  # Adjust based on your setup

def calculate_ground_distance(v, image_height, focal_length_px, camera_height):
    # Calculate the vertical offset from the image center
    v_center = image_height / 2
    pixel_offset = v - v_center

    # Calculate the angle of depression in radians
    theta = math.atan(pixel_offset / focal_length_px)

    # Calculate distance using trigonometry
    if theta > 0:  # Avoid division by zero
        distance = ((camera_height / math.tan(theta))*2.4)
        return distance
    else:
        return None  # Invalid angle

def detect_objects(frame):
    # Perform inference using YOLO
    results = model(frame)

    # Iterate through each detection result
    for result in results:
        for detection in result.boxes:
            # Get the class ID and confidence score
            class_id = int(detection.cls)
            CONFIDENCE = detection.conf.item()
            
            if CONFIDENCE >= 0.4:  # Confidence threshold
                x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Bounding box coordinates
                class_name = model.names[class_id]

                # Use the bottom of the bounding box for ground distance calculation
                object_base_y = y2

                # Calculate the distance to the object using camera height
                distance = calculate_ground_distance(
                    v=object_base_y,
                    image_height=frame.shape[0],
                    focal_length_px=FOCAL_LENGTH_PX,
                    camera_height=CAMERA_HEIGHT
                )

                # Create a label with the calculated distance
                if distance:
                    overlay_text = f'{class_name} {CONFIDENCE:.2f} Distance: {distance:.2f}m'
                    # Draw bounding box and overlay text
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text_y = y1 - 10
                    cv2.putText(frame, overlay_text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    if 1.2 <= distance <= 1.5:
                        if class_name == 'person':
                            cv2.putText(frame, "STOP!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                            stop_motors()
                        else:
                            cv2.putText(frame, "TURN!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                            
                    elif distance <= 1.2 or class_name == 'traffic cone':
                        cv2.putText(frame, "REVERSE!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                        reverse()
    return frame
