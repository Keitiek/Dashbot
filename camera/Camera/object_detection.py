import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO('yolov8n.pt')

# Constants for camera and known object heights
FOCAL_LENGTH = 48  # Focal length in mm (change as per your camera)
SENSOR_HEIGHT_MM = 48  # Sensor height in mm (assumed; change if known)
IMAGE_HEIGHT_PX = 48  # Image height in pixels (adjust if needed)

# Known real-world height of these objects
KNOWN_PERSON_HEIGHT = 1.7  # Average height of a person in meters
KNOWN_TRUCK_HEIGHT = 3.5  # Average height of a truck in meters
KNOWN_TRAFFIC_CONE_HEIGHT = 0.45  # Minimal height of a traffic cone for 30mph-50mph in meters

def detect_objects(frame):
    # Perform inference using YOLO
    results = model(frame)
    
    # Iterate through each detection result
    for result in results:
        for detection in result.boxes:
            # Get the class ID and confidence score
            class_id = int(detection.cls)
            confidence = detection.conf.item()
            
            if confidence >= 0.4:  # Confidence threshold to filter weak detections
                x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Bounding box coordinates
                class_name = model.names[class_id]  # Get class name based on class_id
                
                # Calculate the height of the bounding box in pixels
                object_height_in_image_px = y2 - y1

                # Safeguard: Ensure object_height_in_image_px is not zero or too small
                if object_height_in_image_px <= 0:
                    print(f"Bounding box height too small: {object_height_in_image_px}px. Skipping...")
                    continue

                # Estimate distance using the pinhole camera model formula
                distance = (FOCAL_LENGTH * SENSOR_HEIGHT_MM) / (object_height_in_image_px * IMAGE_HEIGHT_PX)

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
                text_y = y1 - 10  # Adjust text position based on bounding box
                cv2.putText(frame, overlay_text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

                # Begining of the stop if object in front of robot part of the code
                if 0.8 <= distance <= 1.0:
                    if class_name == 'person':
                        cv2.putText(frame, "STOP!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                    else:
                        cv2.putText(frame, "TURN!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                elif 0.6 <= distance <= 0.8 or class_name == 'traffic cone':
                    cv2.putText(frame, "REVERSE!", (250, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)

    return frame
