import cv2
import time
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO(r"C:\Users\SONIA\Desktop\github\model\best.pt")  # Update with the correct path to your weights

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    start_time = time.time()
    
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the current frame
    results = model(frame)

    # Annotate the frame with detection results
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLO Detection", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()