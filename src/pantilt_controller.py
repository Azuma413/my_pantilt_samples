#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np

average_num = 1
POS_LIST = np.zeros((2, average_num), dtype=float)
PANTILT_POS = [0,0]
def callback1(data):
    global POS_LIST
    global PANTILT_POS
    velocity = 1
    target_pos = JointState()
    new_pos = np.array([data.x / 160.0 * velocity, data.y / 120.0 * velocity]).reshape(2,-1)
    POS_LIST = np.hstack((POS_LIST, new_pos))
    POS_LIST = np.delete(POS_LIST, 0, axis=1)
    average_pos = np.average(POS_LIST, axis=1)
    target_pos.position = [PANTILT_POS[0] + average_pos[0], PANTILT_POS[1] + average_pos[1]]
    data_pub.publish(target_pos)

def callback2(data):
    global PANTILT_POS
    PANTILT_POS[0] = data.position[1]
    PANTILT_POS[1] = data.position[0]


if __name__ == "__main__":
    rospy.init_node('pntilt_controller')
    data_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    s1 = rospy.Subscriber('Detected_Object_Position', Point, callback1, queue_size=10)
    s2 = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, callback2, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
