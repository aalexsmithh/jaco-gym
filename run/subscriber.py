import sys, rospy
from gazebo_msgs.msg import LinkStates 


'''
usage:

$ python suscriber.py '/topic/topic' '/topic_msgs/TopicType'
'''

def calc_reward_cupball(cup_state, ball_state, pinky_state):
	# check if below lip of the cup
	# if cup_state[2] - ball_state[2] > 0:
	# 	reward = 0
	# else:
	# 	rwrd = [(x-y)**2 for x,y in zip(cup_state, ball_state)]
	# 	reward = 3/sum(rwrd)
	

	rwrd = [(x-y)**2 for x,y in zip(cup_state, pinky_state)]
	# reward -= (sum(rwrd)/3)

	print sum(rwrd)/3

def cb1(msg):
	i = msg.name.index('kendama::base_link')
	cup = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]
	i = msg.name.index('kendama::ball')
	ball = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]
	i = msg.name.index('jaco_on_table::jaco_8_finger_pinkie')
	pinky = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]
	calc_reward_cupball(cup, ball, pinky)

def cb(msg):
	i = msg.name.index('kendama::base_link')
	print 'x: {} \t y: {} \t z: {}'.format(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z)

sub = rospy.Subscriber('/gazebo/link_states', LinkStates, cb1)
rospy.init_node('echo')

while not rospy.is_shutdown():
	rospy.spin()