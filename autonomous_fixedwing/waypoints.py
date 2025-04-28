import time
import json
from pymavlink import mavutil

# Function to send MAVLink commands
def send_command_long(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        param1, param2, param3, param4, param5, param6, param7
    )

# Connect to the drone
master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')
master.wait_heartbeat()
print("✅ Heartbeat received. Connected to vehicle.")

# **Step 1: Arm the drone**
send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
print("🛠 Arming the drone...")
time.sleep(2)

# **Step 2: Set Airspeed for Fixed-Wing**
airspeed = 15  # m/s
send_command_long(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, airspeed)
print(f"⚡ Setting airspeed to {airspeed} m/s...")

# **Step 3: Takeoff Command**
takeoff_alt = 10  # meters
send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, takeoff_alt)
print(f"🚀 Takeoff initiated to {takeoff_alt}m altitude...")

# **Step 4: Confirm takeoff success**
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg.alt >= takeoff_alt * 1000:  # Convert meters to mm
        print(f"✅ Takeoff confirmed. Current altitude: {msg.alt / 1000} m")
        break
    time.sleep(1)

# **Step 5: Load waypoints from locwaypoints.txt**
with open("locwaypoints.txt", "r") as file:
    waypoints = json.load(file)

# **Step 6: Send Mission Count**
master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
print(f"📍 Sending {len(waypoints)} waypoints to vehicle...")

# **Step 7: Send Each Waypoint Upon Request**
for i, waypoint in enumerate(waypoints):
    lat = waypoint['latitude']
    lon = waypoint['longitude']
    alt = 10  # Altitude for waypoints

    while True:
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True)
        if msg.seq == i:
            break
    
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        i,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        lat, lon, alt
    )
    print(f"✅ Waypoint {i} sent: lat={lat}, lon={lon}, alt={alt}")
    time.sleep(1)

# **Step 8: Confirm Waypoint Upload Success**
while True:
    msg = master.recv_match(type='MISSION_ACK', blocking=True)
    if msg:
        print("📜 Mission acknowledged by vehicle.")
        break

# **Step 9: Set First Waypoint**
master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
print("🏁 Setting first waypoint as current...")

# **Step 10: Set Mode to AUTO**
mode = 'AUTO'
mode_id = master.mode_mapping().get(mode)
if mode_id is not None:
    master.set_mode(mode_id)
    print("🚦 Setting mode to AUTO...")
else:
    print("❌ AUTO mode not available!")

time.sleep(2)

# **Step 11: Monitor Mission Progress**
print("🛫 Following waypoints...")
while True:
    msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
    if msg and msg.seq == len(waypoints) - 1:
        print("🎯 Final waypoint reached.")
        break
    time.sleep(1)

# **Step 12: Return to Launch (RTL)**
mode = 'RTL'
mode_id = master.mode_mapping().get(mode)
if mode_id is not None:
    master.set_mode(mode_id)
    print("🔄 Setting mode to RTL...")
else:
    print("❌ RTL mode not available!")

time.sleep(2)

# **Step 13: Wait for Drone to Land**
print("🛬 Waiting for drone to land...")
while True:
    msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
    if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
        print("✅ Drone has landed.")
        break
    time.sleep(1)

# **Step 14: Disarm the Drone**
send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
print("🛑 Drone disarmed. Mission complete.")

# **Cleanup**
master.close()
