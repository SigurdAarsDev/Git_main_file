from pymavlink import mavutil
import time

# Connect to ArduSub
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
time.sleep(2)
print("Armed")

# Send PWM to PWM Output 1 (AUX1)
# Typical safe PWM range for ArduSub: 1100 - 1900
for pwm in [0]:#, 0]:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        1,      # Servo channel 1 (PWM Output 1)
        pwm,    # PWM value
        0,0,0,0,0
    )
    print(f"PWM sent to channel 1: {pwm}")

    end_time = time.time() + 1
    while time.time() < end_time:
        time.sleep(0.1)  # small chunks
    print("Done")