import time
from pymavlink import mavutil

# Connect to the vehicle (TCP for SITL or Mission Planner)
master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

# Wait for heartbeat to confirm connection
master.wait_heartbeat()
print("✅ Heartbeat received. Connected to the drone.")

# Arm the drone
def arm():
    print("🔧 Arming the drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1, 0, 0, 0, 0, 0, 0  # Arm
    )
    time.sleep(2)

# Set flight mode
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print(f"🔁 Mode set to {mode}")
    time.sleep(2)

# Takeoff to 10 meters
def takeoff(altitude):
    print(f"🛫 Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(10)

# Land the drone
def land():
    print("🛬 Initiating landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(10)

try:
    arm()
    set_mode("GUIDED")
    takeoff(10)
    land()

    # Wait until the drone has landed (optional)
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            print("✅ Drone has landed and disarmed.")
            break
        time.sleep(1)

except Exception as e:
    print("❌ Error:", e)

finally:
    master.close()
    print("🔌 Connection closed.")
