import gym
import os
import rospy
import roslaunch
import subprocess
import time
import numpy as np

from gym import error, spaces, utils
from gym.utils import seeding

from jaco_gym.envs import gazebo_env

from sensor_msgs.msg import Image, JointState
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

from std_srvs.srv import Empty

class JacoEnv(gazebo_env.GazeboEnv):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		# run the jaco-arm launch file
		gazebo_env.GazeboEnv.__init__(self, "JacoArmWithBallAndCup_v0.launch")
		time.sleep(10) # Wait for gzserver to launch

		# action space
		self.joint_0_pub = rospy.Publisher('/jaco/jaco_arm_0_joint_position_controller/command', Float64, queue_size=10)
		self.joint_1_pub = rospy.Publisher('/jaco/jaco_arm_1_joint_position_controller/command', Float64, queue_size=10)
		self.joint_2_pub = rospy.Publisher('/jaco/jaco_arm_2_joint_position_controller/command', Float64, queue_size=10)
		self.joint_3_pub = rospy.Publisher('/jaco/jaco_arm_3_joint_position_controller/command', Float64, queue_size=10)
		self.joint_4_pub = rospy.Publisher('/jaco/jaco_arm_4_joint_position_controller/command', Float64, queue_size=10)
		self.joint_5_pub = rospy.Publisher('/jaco/jaco_arm_5_joint_position_controller/command', Float64, queue_size=10)

		self.finger_0_pub = rospy.Publisher('/jaco/jaco_finger_joint_0_position_controller/command', Float64, queue_size=10)
		self.finger_2_pub = rospy.Publisher('/jaco/jaco_finger_joint_2_position_controller/command', Float64, queue_size=10)
		self.finger_4_pub = rospy.Publisher('/jaco/jaco_finger_joint_4_position_controller/command', Float64, queue_size=10)

		self.pubs = [self.joint_0_pub, self.joint_1_pub, self.joint_2_pub, self.joint_3_pub, self.joint_4_pub, self.joint_5_pub, self.finger_0_pub, self.finger_2_pub, self.finger_4_pub, self.joint_0_pub]

		# state space
		self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.state_sub = rospy.Subscriber('/jaco/joint_states', JointState, self.state_cb)

		rospy.init_node('gym_gazebo')
		# self.joint_0_sub = rospy.Subscriber('/jaco/jaco_arm_0_joint_position_controller/state', JointControllerState, self.joint_0_cb)
		# self.joint_1_sub = rospy.Subscriber('/jaco/jaco_arm_1_joint_position_controller/state', JointControllerState, self.joint_1_cb)
		# self.joint_2_sub = rospy.Subscriber('/jaco/jaco_arm_2_joint_position_controller/state', JointControllerState, self.joint_2_cb)
		# self.joint_3_sub = rospy.Subscriber('/jaco/jaco_arm_3_joint_position_controller/state', JointControllerState, self.joint_3_cb)
		# self.joint_4_sub = rospy.Subscriber('/jaco/jaco_arm_4_joint_position_controller/state', JointControllerState, self.joint_4_cb)
		# self.joint_5_sub = rospy.Subscriber('/jaco/jaco_arm_5_joint_position_controller/state', JointControllerState, self.joint_5_cb) 

		# self.finger_0_sub = rospy.Subscriber('/jaco/jaco_finger_joint_0_position_controller/state', JointControllerState, self.finger_0_cb)
		# self.finger_2_sub = rospy.Subscriber('/jaco/jaco_finger_joint_2_position_controller/state', JointControllerState, self.finger_2_cb)
		# self.finger_4_sub = rospy.Subscriber('/jaco/jaco_finger_joint_4_position_controller/state', JointControllerState, self.finger_4_cb)

		# self.joint_0_state = 0.0
		# self.joint_1_state = 0.0
		# self.joint_2_state = 0.0
		# self.joint_3_state = 0.0
		# self.joint_4_state = 0.0
		# self.joint_5_state = 0.0
		# self.finger_0_state = 0.0
		# self.finger_2_state = 0.0
		# self.finger_4_state = 0.0


		# # gym init settings
		# #### THIS WILL NEED TO BE CHANGED TO A DIFFERENT SPACE!!!!!!!!!!!!!!!!!!!!!!!!!
		# self.action_space = spaces.Discrete(3) #F,L,R
		# self.reward_range = (-np.inf, np.inf)

		# # gazebo init commands
		self.gazebo_step_size = long(200)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)


	def _step(self, action):
		#control the robot based on the action var
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		for i, joint in enumerate(self.pubs):
			joint.publish(Float64(action[i]))
		######### DO WORK

		for i,pub in enumerate(self.pubs):
			pub.publish(Float64(action[i]))

		######### END OF WORK
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			#resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")
		return self.state

	def _reset(self):
		# Resets the state of the environment and returns an initial observation.
		rospy.wait_for_service('/gazebo/reset_world')
		try:
			#reset_proxy.call()
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_world service call failed")

		# THIS MUST BE DEFINED. IMAGE SPACE? OR DISCRITIZED ANGLES
		state = None
		return state

	def _pause(self, msg):
		programPause = raw_input(str(msg))

	def _render(self, mode='human', close=False):
		# Not sure if this is necessary to define as we are using gazebo for rendering.
		pass

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def state_cb(self, msg):
		self.state = msg.position
		# for i, joint in enumerate(msg.position):
		# 	self.state[i] = joint

	def calc_reward(self, gz_states):
		pass

if __name__ == '__main__':
	try:
		t = JacoEnv()
	except KeyboardInterrupt as e:
		t._close()
