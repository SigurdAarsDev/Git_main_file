from pymavlink import mavutil
import time

# Connect
master = mavutil.mavlink_connection('/dev/ttyACM0')
master.wait_heartbeat()
print("Connected to ArduSub")

# Arm the vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 1 = arm
    0,0,0,0,0,0
)
time.sleep(2)  # wait for ESCs to start arming

# Initialize ESC safely with low PWM first
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    1,      # PWM Output 1 / Thruster 1
    1100,   # Low PWM for safe ESC init
    0,0,0,0,0
)
time.sleep(1)

# Then ramp to neutral
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    1,
    1480,   # Neutral PWM
    0,0,0,0,0
)
time.sleep(2)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    1,
    0,   # Neutral PWM
    0,0,0,0,0
)
print("Thruster 1 initialized safely, beeping should stop")