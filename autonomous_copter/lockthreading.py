import cv2
import time
import json
import numpy as np
import threading
from pymavlink import mavutil
from ultralytics import YOLO


model = YOLO('/home/pi/models/best.pt')  # Change Path Accordingly

current_waypoint = 0
ml_active = False
lock = threading.Lock()

def send_command_long(master, command, param1, param2, param3, param4, param5, param6, param7):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        param1, param2, param3, param4, param5, param6, param7
    )

def waypoint_mission():
    global current_waypoint, ml_active
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
    master.wait_heartbeat()
    print("Connected to vehicle for waypoint mission.")

    with open("locwaypoints.txt", "r") as file:
        waypoints = json.load(file)

    for i, waypoint in enumerate(waypoints):
        lat, lon, alt = waypoint['latitude'], waypoint['longitude'], 10

    
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        with lock:
            current_waypoint = i
            ml_active = 3 <= i <= 6 
        print(f"Waypoint {i} sent: lat={lat}, lon={lon}, alt={alt}, ML active: {ml_active}")
        time.sleep(2)

    master.close()

def zigzag_surveillance():
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
    master.wait_heartbeat()
    print("Connected to vehicle for zigzag surveillance.")

    with open("coordinates.txt", "r") as file:
        coordinates = json.load(file)

    boundary_lat = [point['latitude'] for point in coordinates]
    boundary_long = [point['longitude'] for point in coordinates]
    lat_min, lat_max = min(boundary_lat), max(boundary_lat)
    long_min, long_max = min(boundary_long), max(boundary_long)

    num_lines = 10
    lat_points = np.linspace(lat_min, lat_max, num_lines)

    for i, lat in enumerate(lat_points):
        lon = long_min if i % 2 == 0 else long_max
        send_command_long(master, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, lat, lon, 10)
        print(f"Zigzag Waypoint {i} sent: lat={lat}, lon={lon}")
        time.sleep(2)

    master.close()

def ml_detection():
    global ml_active
    print("Starting ML detection...")
    cap = cv2.VideoCapture(0)

    while True:
        with lock:
            if not ml_active:
                time.sleep(0.5)
                continue

        ret, frame = cap.read()
        if not ret:
            break

        start_time = time.time()
        results = model(frame)
        annotated_frame = results[0].plot()

        fps = 1 / (time.time() - start_time)
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        print(f"FPS: {fps:.2f}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("ML detection completed.")


waypoint_thread = threading.Thread(target=waypoint_mission)
zigzag_thread = threading.Thread(target=zigzag_surveillance)
ml_thread = threading.Thread(target=ml_detection)

waypoint_thread.start()
waypoint_thread.join()

zigzag_thread.start()
ml_thread.start()

zigzag_thread.join()
ml_thread.join()

print("All tasks completed successfully.")
