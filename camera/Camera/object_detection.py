import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO('yolov8n.pt')

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
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add text label with class name and confidence
                label = f"{class_name}: {confidence:.2f}"
                
                # Position the text above the bounding box, adjusted by a fixed offset
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
                text_x = x1
                text_y = y1 - 10  # Position the text slightly above the box
                
                # Draw background rectangle for text to make it more readable
                cv2.rectangle(frame, (text_x, text_y - text_size[1]), (text_x + text_size[0], text_y), (0, 255, 0), -1)
                # Put the text on the frame
                cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    return frame
