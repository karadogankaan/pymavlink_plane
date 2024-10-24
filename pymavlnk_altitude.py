from pymavlink import mavutil
#import sys
import time

master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))


master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(msg)
    if msg:
        altitude = msg.relative_alt / 1000.0
        print("Altitude: %.2f meters" % altitude)
        break


master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    1,  # confirmation
    1,  # param1 (1 to indicate arm)
    0,  # param2 (unused)
    0,  # param3 (unused)
    0,  # param4 (unused)
    0,  # param5 (latitude, unused)
    0,  # param6 (longitude, unused)
    0  # param7 (altitude, unused)
)

print("Arm command sent!")


while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        print(msg)
        break



time.sleep(5)


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    1,  # confirmation
    0,  # param1 (unused)
    0,  # param2 (unused)
    0,  # param3 (unused)
    0,  # param4 (unused)
    0,  # param5 (latitude, unused)
    0,  # param6 (longitude, unused)
    700  # param7 (altitude)
    )


while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        print(msg)
        break


master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        altitude = msg.relative_alt / 1000.0
        print("Altitude: %.2f meters" % altitude)