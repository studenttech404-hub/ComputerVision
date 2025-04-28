import time
import json
import numpy as np
from pymavlink import mavutil

# Function to read coordinates from a file
def read_coordinates_from_file(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    return data

# Function to send commands to the drone
def send_command_long(command, param1, param2, param3, param4, param5, param6, param7):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        param1, param2, param3, param4, param5, param6, param7
    )

# Connecting to the drone
master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

try:
    master.wait_heartbeat()
    print("Heartbeat received. Connected to the vehicle.")

    # Arming the drone
    send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    print("Arming the drone...")
    time.sleep(2)

    # Set mode to TAKEOFF
    mode = 'TAKEOFF'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to TAKEOFF...")
    time.sleep(10)  # Allow some time for the fixed-wing to take off

    # Load waypoints from locwaypoints.txt for the first mission
    waypoints_file = "locwaypoints.txt"
    waypoints = read_coordinates_from_file(waypoints_file)

    # Load coordinates from the file for the second mission
    coordinates_file = "coordinates4ctol.txt"
    coordinates = read_coordinates_from_file(coordinates_file)

    boundary_lat = [point['latitude'] for point in coordinates]
    boundary_long = [point['longitude'] for point in coordinates]

#  # Close the boundary (if needed)
#     boundary_lat.append(boundary_lat[0])
#     boundary_long.append(boundary_long[0])

    # Calculate zigzag waypoints for the second mission
    lat_min, lat_max = min(boundary_lat), max(boundary_lat)
    long_min, long_max = min(boundary_long), max(boundary_long)
    num_lines = 10
    lat_points = np.linspace(lat_min, lat_max, num_lines)
    for i, lat in enumerate(lat_points):
        if i % 2 == 0:
            waypoints.append({'latitude': lat, 'longitude': long_min})
            waypoints.append({'latitude': lat, 'longitude': long_max})
        else:
            waypoints.append({'latitude': lat, 'longitude': long_max})
            waypoints.append({'latitude': lat, 'longitude': long_min})

    # Send mission count for all waypoints
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    print(f"Total mission count sent: {len(waypoints)}")

    # Send all waypoints
    for i, waypoint in enumerate(waypoints):
        lat = waypoint['latitude']
        lon = waypoint['longitude']
        alt = 10  # Consistent altitude can be adjusted if necessary
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        print(f"Sending waypoint {i}: lat={lat}, lon={lon}, alt={alt}")
        time.sleep(1)

    # Set the first waypoint as current
    master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
    print("First waypoint set as current...")

    # Set mode to AUTO to start following all waypoints
    mode = 'AUTO'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Mode set to AUTO...")
    time.sleep(2)

    # Wait for the drone to complete all waypoints
    print("Following all waypoints...")
    while True:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
        if msg and msg.seq == len(waypoints) - 1:
            print("Final waypoint reached.")
            break
        time.sleep(1)

    # Set mode to RTL (Return to Launch) after mission completion
    mode = 'RTL'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to RTL...")
    time.sleep(2)

    print("Waiting for the drone to return and land...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg and msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            print("Drone has landed.")
            break
        time.sleep(1)

except Exception as e:
    print("Error:", e)

finally:
    master.close()
