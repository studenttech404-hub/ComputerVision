from pymavlink import mavutil

# Connect to the Pixhawk via USB serial
# Replace '/dev/ttyACM0' if your device shows up differently
connection_string = '/dev/ttyACM1'
baud_rate = 115200

try:
    print(f"🔌 Connecting to Pixhawk on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

    # Wait for the first heartbeat 
    print("⏳ Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"✅ Heartbeat received from system (ID {master.target_system}), component (ID {master.target_component})")

except Exception as e:
    print("❌ Connection failed:", e)

finally:
    if 'master' in locals():
        master.close()
        print("🔌 Connection closed.")
