import cv2
from ultralytics import YOLO
import math

# Load YOLO model
model = YOLO('yolov8n.pt')

# Constants for camera and known object heights
FOCAL_LENGTH_PX = 48  # Focal length in mm (change as per your camera)
SENSOR_HEIGHT_MM = 48  # Sensor height in mm (assumed; change if known)
IMAGE_HEIGHT_PX = 48  # Image height in pixels (adjust if needed)

# Known real-world height of these objects
KNOWN_PERSON_HEIGHT = 1.7  # Average height of a person in meters
KNOWN_TRUCK_HEIGHT = 3.5  # Average height of a truck in meters
KNOWN_TRAFFIC_CONE_HEIGHT = 0.45  # Minimal height of a traffic cone for 30mph-50mph in meters

# Height of the camera from the ground in meters
CAMERA_HEIGHT = 0.81  # Adjust based on your setup

def calculate_ground_distance(v, image_height, focal_length_px, camera_height):
    """
    Calculate ground distance using camera height and pixel position.

    Args:
    - v: Vertical pixel coordinate of the object's base.
    - image_height: Total height of the image in pixels.
    - focal_length_px: Focal length of the camera in pixels.
    - camera_height: Height of the camera from the ground in meters.

    Returns:
    - Distance to the object in meters.
    """
    # Calculate the vertical offset from the image center
    v_center = image_height / 2
    pixel_offset = v - v_center

    # Calculate the angle of depression in radians
    theta = math.atan(pixel_offset / focal_length_px)

    # Calculate distance using trigonometry
    if theta > 0:  # Avoid division by zero
        distance = (camera_height / math.tan(theta))
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
            confidence = detection.conf.item()
            
            if confidence >= 0.4:  # Confidence threshold
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
                    overlay_text = f'{class_name} {confidence:.2f} Distance: {distance:.2f}m'
                    # Draw bounding box and overlay text
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text_y = y1 - 10
                    cv2.putText(frame, overlay_text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

    return frame
