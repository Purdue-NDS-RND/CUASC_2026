from pymavlink import mavutil
import subprocess, datetime

# Change port to wherever your FC is connected
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

print("Waiting for GPS time...")
while True:
    msg = master.recv_match(type='SYSTEM_TIME', blocking=True)
    if msg and msg.time_unix_usec > 0:
        unix_sec = msg.time_unix_usec / 1e6
        dt = datetime.datetime.utcfromtimestamp(unix_sec)
        time_str = dt.strftime('%Y-%m-%d %H:%M:%S')
        subprocess.run(['sudo', 'date', '-s', time_str])
        print(f"Time set to: {time_str}")
        break
