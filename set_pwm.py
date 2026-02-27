#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandLongRequest

def set_servo_pwm(servo_number, pwm_value):
    """
    Sends MAV_CMD_DO_SET_SERVO to Pixhawk

    servo_number: Output channel number (1-16 typically)
    pwm_value: PWM value in microseconds (1000–2000 typical)
    """

    rospy.wait_for_service('/mavros/cmd/command')

    try:
        command_service = rospy.ServiceProxy(
            '/mavros/cmd/command',
            CommandLong
        )

        req = CommandLongRequest()
        req.broadcast = False
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = 0

        req.param1 = servo_number   # Servo channel number
        req.param2 = pwm_value      # PWM value
        req.param3 = 0
        req.param4 = 0
        req.param5 = 0
        req.param6 = 0
        req.param7 = 0

        response = command_service(req)

        if response.success:
            rospy.loginfo("PWM command sent successfully.")
        else:
            rospy.logwarn("Failed to send PWM command.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("set_pwm_node")

    # Example: Set SERVO9 to 1500µs
    set_servo_pwm(9, 1500)