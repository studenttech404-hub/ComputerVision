import time
import json
from pymavlink import mavutil

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

    # Set mode to TAKEOFF to start the takeoff process
    mode = 'TAKEOFF'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to TAKEOFF...")
    time.sleep(10)  # Allow some time for the fixed-wing to take off

    # Load waypoints from locwaypoints.txt
    with open("locwaypoints.txt", "r") as file:
        waypoints = json.load(file)

    # Send each waypoint to the vehicle
    for i, waypoint in enumerate(waypoints):
        lat = waypoint['latitude']
        lon = waypoint['longitude']
        alt = 10  # Altitude for waypoints (can be adjusted)
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        print(f"Waypoint {i} sent: lat={lat}, lon={lon}, alt={alt}")
        time.sleep(1)

    # Send mission count
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    print(f"Mission count sent: {len(waypoints)}")

    # Wait for mission acknowledgment
    while True:
        msg = master.recv_match(type='MISSION_ACK', blocking=True)
        if msg:
            print("Mission acknowledged by the vehicle.")
            break

    # Set the first waypoint as current
    master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
    print("Setting first waypoint as current...")

    # Set mode to AUTO to start following waypoints
    mode = 'AUTO'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to AUTO...")
    time.sleep(2)

    # Wait for the drone to complete the mission
    print("Following waypoints...")
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
