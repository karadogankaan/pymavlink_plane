from pymavlink import mavutil

#UDP Bağlantısı
connection_string = 'udpin:127.0.0.1:14550'
master = mavutil.mavlink_connection(connection_string) 

#Heartbeat sinyali tanımlama
master.wait_heartbeat()
print("Heartbeat gonderiliyor..")

#ARM etme komutu
def arm_vehicle():
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    
    master.motors_armed_wait()
    print("Cihaz Arm.")

arm_vehicle()

