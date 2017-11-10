#!/usr/bin/env python
# license removed for brevity
import rospy, math
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

joints = ['/jaco/jaco_arm_0_joint_position_controller/command',\
            '/jaco/jaco_arm_1_joint_position_controller/command',\
            '/jaco/jaco_arm_2_joint_position_controller/command',\
            '/jaco/jaco_arm_3_joint_position_controller/command',\
            '/jaco/jaco_arm_4_joint_position_controller/command',\
            '/jaco/jaco_finger_joint_0_position_controller/command',\
            '/jaco/jaco_finger_joint_2_position_controller/command',\
            '/jaco/jaco_finger_joint_4_position_controller/command']

def spinner():
    pubs = [rospy.Publisher(joint, Float64, queue_size=10) for joint in joints]
    pu = rospy.Publisher('/jaco/jaco_arm_5_joint_position_controller/command', Float64, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    # rospy.init_node('/jaco/jaco_arm_5_joint_position_controller/command', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz

    x = 0 
    while not rospy.is_shutdown():
        for pub in pubs:
            pub.publish(Float64(0))
        pos = Float64(math.sin(x))
        # rospy.loginfo(hello_str)
        pu.publish(pos)
        rate.sleep()
        x += 0.05

if __name__ == '__main__':
    try:
        spinner()
    except rospy.ROSInterruptException:
        pass