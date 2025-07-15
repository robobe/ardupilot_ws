from pymavlink import mavutil
import time

# Connect to MAVLink
# master = mavutil.mavlink_connection('udp:localhost:14550')
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

def current_time_us():
    """Return current time in microseconds"""
    return int(time.time() * 1_000_000)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Send initial TIMESYNC message to start sync loop (optional)
master.mav.timesync_send(0, current_time_us())

while True:
    msg = master.recv_match(type='TIMESYNC', blocking=True, timeout=5)
    if msg is None:
        print("No TIMESYNC message received")
        continue

    print(f"Received TIMESYNC: tc1={msg.tc1}, ts1={msg.ts1}")

    if msg.tc1 == 0:
        # Autopilot is requesting time sync — reply with our time
        master.mav.timesync_send(0, current_time_us())
        print("Responded with timesync")
    else:
        # Autopilot responded to our request, calculate offset
        now = current_time_us()
        rtt = now - msg.ts1
        offset = msg.tc1 + rtt // 2 - now
        print(f"RTT: {rtt} µs, Time offset estimate: {offset} µs")
