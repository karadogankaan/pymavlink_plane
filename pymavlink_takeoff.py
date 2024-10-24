from pymavlink import mavutil
import sys

master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
master.wait_heartbeat()



print("baglandi")


mode = 'GUIDED'


if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))

    sys.exit(1)


mode_id = master.mode_mapping()[mode]


master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print("komut verildi")
master.motors_armed_wait()

print("arm gerceklesti")


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10)

altitude=0

while altitude <= 9.5 :
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'HEARTBEAT':
        msg1 = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
        print(msg1)
        altitude = msg1.relative_alt/1000
        print(altitude)

print(" yukselis bitti, land moduna geciliyor.")

mode1 = 'LAND'


if mode1 not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode1))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)


mode_id = master.mode_mapping()[mode1]


master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print(" LAND moduna gecis yapildi, inis basliyor.")

while altitude >= 0.5 :
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'HEARTBEAT':
        msg1 = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
        print(msg1)
        altitude = msg1.relative_alt/1000
        print(altitude)

print(" inis basariyla gerceklesti. disarm oluyor.")


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

print(" disarm oldu.")
