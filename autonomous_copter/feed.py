import cv2
import time
import json
import numpy as np
import threading
import queue
from pymavlink import mavutil
from ultralytics import YOLO

model = YOLO('/home/pi/model/best.pt')  # Update this path for Raspberry Pi


frame_queue = queue.Queue(maxsize=1)
annotated_queue = queue.Queue(maxsize=1)

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("Connected to Pixhawk via USB.")


def send_waypoint(lat, lon, alt=10):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        0, 0, 0, 0,
        lat, lon, alt
    )

def zigzag_surveillance():
    print("Starting Zigzag Surveillance...")
    with open("coordinates.txt", "r") as file:
        coordinates = json.load(file)

    latitudes = [pt['latitude'] for pt in coordinates]
    longitudes = [pt['longitude'] for pt in coordinates]
    lat_min, lat_max = min(latitudes), max(latitudes)
    lon_min, lon_max = min(longitudes), max(longitudes)

    num_lines = 10
    lat_lines = np.linspace(lat_min, lat_max, num_lines)

    for i, lat in enumerate(lat_lines):
        lon = lon_min if i % 2 == 0 else lon_max
        send_waypoint(lat, lon)
        print(f"Sent Waypoint {i}: lat={lat}, lon={lon}")
        time.sleep(1)

    print("Zigzag Surveillance Completed.")


def camera_feed():
    print("Camera Feed Started...")
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        if not frame_queue.full():
            frame_queue.put(frame)

        if not annotated_queue.empty():
            output_frame = annotated_queue.get()
            cv2.imshow("Live YOLO Detection", output_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

def ml_detection():
    print("YOLOv8 Detection Thread Started...")
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            start_time = time.time()

            results = model(frame)
            annotated = results[0].plot()

            fps = 1 / (time.time() - start_time)
            cv2.putText(annotated, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if not annotated_queue.full():
                annotated_queue.put(annotated)


if __name__ == '__main__':
    t1 = threading.Thread(target=zigzag_surveillance)
    t2 = threading.Thread(target=camera_feed)
    t3 = threading.Thread(target=ml_detection)

    t1.start()
    t2.start()
    t3.start()

    t1.join()
    t2.join()
    t3.join()

    print("All operations completed.")