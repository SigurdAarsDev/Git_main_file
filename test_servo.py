from pymavlink import mavutil
import time

# Connect to SITL
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to SITL")

# Arm vehicle
master.arducopter_arm()
master.motors_armed_wait()
print("Armed")

# Disable SERVO9 function (optional safety)
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'SERVO9_FUNCTION',
    0,
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)

time.sleep(1)

# Send PWM to SERVO9
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    9,        # servo number
    1900,     # pwm
    0,0,0,0,0
)

print("PWM sent")
time.sleep(5)