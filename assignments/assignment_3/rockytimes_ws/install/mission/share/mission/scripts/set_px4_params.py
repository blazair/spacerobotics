#!/usr/bin/env python3
import time
from pymavlink import mavutil

# Connect to the SITL instance (adjust the connection string as needed)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# List of parameter commands
params = {
    'COM_ARM_WO_GPS': 1,
    'COM_ARM_IMU_ACC': 20.0,
    'COM_ARM_IMU_GYR': 0.1,
    'COM_ARM_EKF_HGT': 0.1,
    'COM_ARM_EKF_POS': 0.1,
    'COM_ARM_EKF_VEL': 0.1,
    'CBRK_GPSFAIL': 240024,
}

# Send each parameter
for param, value in params.items():
    print(f"Setting {param} to {value}")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)

print("Parameters set successfully!")
