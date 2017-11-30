import gym
import os
import rospy
import roslaunch
import subprocess
import time
import numpy as np
import math

from gym import error, spaces, utils
from gym.utils import seeding

from jaco_gym.envs import gazebo_env

from sensor_msgs.msg import Image, JointState
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates

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
		self.joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.link_states = None
		self.state_sub = rospy.Subscriber('/jaco/joint_states', JointState, self.state_cb)
		self.gz_states = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gz_state_cb)

		rospy.init_node('gym_gazebo')

		# # gym init settings
		# #### THIS WILL NEED TO BE CHANGED TO A DIFFERENT SPACE!!!!!!!!!!!!!!!!!!!!!!!!!
		self.max_joint_pos = math.pi
		# self.action_space = spaces.Box(low=-self.max_joint_pos, high=self.max_joint_pos) #F,L,R
		# self.reward_range = (0, np.inf)
		self.goal = None
		self.reward = 0.0

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

		self.calc_reward()

		######### END OF WORK
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			#resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		### check if the task is done
		if self.reward < 1e-3:
			done = True
		#check self collision physics??

		return self.joint_state, self.reward, done, {}

	def _reset(self):
		# Resets the state of the environment and returns an initial observation.
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			#reset_proxy.call()
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_simulation service call failed")

		# Unpause simulation to make observation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			#resp_pause = pause.call()
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		########## NECESSARY TO RESET THE self.joint_states VARIABLE

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			#resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		return self.joint_state

	def _pause(self, msg):
		programPause = raw_input(str(msg))

	def _render(self, mode='human', close=False):
		# Not sure if this is necessary to define as we are using gazebo for rendering.
		pass

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def state_cb(self, msg):
		self.joint_state = msg.position
		# for i, joint in enumerate(msg.position):
		# 	self.state[i] = joint

	def gz_state_cb(self, msg):
		i = msg.name.index('jaco_on_table::jaco_8_finger_pinkie')
		self.link_states = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]

	def calc_reward(self):
		if self.goal is not None:
			# MSE of link_state from goal
			rwrd = [(x-y)**2 for x,y in zip(self.goal, self.link_states)]
			self.reward = sum(rwrd)/3

	def set_goal(self, goal):
		self.goal = goal

if __name__ == '__main__':
	try:
		t = JacoEnv()
	except KeyboardInterrupt as e:
		t._close()
