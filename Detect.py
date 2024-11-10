import cv2
import serial
import time
from ultralytics import YOLO

# Set up the serial connection to Arduino
ser = serial.Serial('COM6', 9600, timeout=1)  # Replace 'COM6' with your port
time.sleep(2)  # Wait for the connection to establish

# Load the YOLO model
model = YOLO('best.pt')  # Ensure 'best.pt' is in the correct directory

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

# Function to send angle command to Arduino
def send_angle_command(base_angle, shoulder_angle, elbow_angle, wrist_angle):
    """Send servo angles to Arduino."""
    command = f"{base_angle},{shoulder_angle},{elbow_angle},{wrist_angle}\n"
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")

    response = ser.readline().decode().strip()
    if response:
        print(f"Arduino Response: {response}")

def detect_brinjal():
    """Open the camera and perform brinjal detection continuously."""
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Perform object detection using YOLO
        results = model(frame)

        for result in results:
            boxes = result.boxes  # Bounding boxes
            for box in boxes:
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Extract box coordinates
                confidence = box.conf[0]  # Confidence score

                if confidence > 0.7:  # Only act on high-confidence detections
                    center_x = (x_min + x_max) // 2
                    center_y = (y_min + y_max) // 2
                    z = 100.0  # Example z-coordinate (can be expanded with more logic)

                    print(f"Brinjal detected at: ({center_x}, {center_y}, {z})")

                    # Calculate angles for servos based on object position
                    base_angle = 100 - int(center_x * (180 / CAMERA_WIDTH))
                    shoulder_angle = 125 - int(center_y * (100 / CAMERA_HEIGHT))
                    elbow_angle = 48 - int(center_y * (90 / CAMERA_HEIGHT))
                    wrist_angle = 60

                    # Send the angle commands to Arduino
                    send_angle_command(base_angle, shoulder_angle, elbow_angle, wrist_angle)

                    # Draw bounding box and label
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                    label_text = f"Brinjal: {confidence:.2f}"
                    cv2.putText(frame, label_text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show the frame with detections
        cv2.imshow('Brinjal Detection Feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

try:
    detect_brinjal()
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    ser.close()
