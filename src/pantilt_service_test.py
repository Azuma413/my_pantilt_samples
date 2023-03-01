#!/usr/bin/env python
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from geometry_msgs.msg import Twist

#rosrun turtlesim turtle_teleop_key

def callback1(data):
    servo_data1 = DynamixelCommand()
    servo_data2 = DynamixelCommand()
    servo_data1.command = ""
    servo_data2.command = ""
    servo_data1.id = 1
    servo_data2.id = 2
    servo_data1.addr_name = "Goal_Position"
    servo_data2.addr_name = "Goal_Position"
    servo_data1.value = data.linear.x
    servo_data2.value = data.angular.z
    result1 = s_cliant(servo_data1)
    result2 = s_cliant(servo_data2)
    rospy.loginfo("servo1:{}, servo2:{}".format(result1, result2))

if __name__ == "__main__":
    rospy.init_node('pantilt_service_test')
    rospy.wait_for_service('pantilt_service_test')
    s_cliant = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    sub = rospy.Subscriber('turtlebot_telop_keyboard/cmd_vel', Twist, callback1, queue_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
