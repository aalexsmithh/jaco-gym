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

from sensor_msgs.msg import Image
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

from std_srvs.srv import Empty

class JacoEnv(gazebo_env.GazeboEnv):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		# run the jaco-arm launch file
		gazebo_env.GazeboEnv.__init__(self, "JacoArmWithBallAndCup_v0.launch")

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

		# gym init settings
		#### THIS WILL NEED TO BE CHANGED TO A DIFFERENT SPACE!!!!!!!!!!!!!!!!!!!!!!!!!
		self.action_space = spaces.Discrete(3) #F,L,R
		self.reward_range = (-np.inf, np.inf)

		# gazebo init commands
		self.gazebo_step_size = long(200)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

		time.sleep(10) # Wait for gzserver to launch


	def _step(self, action):
		#control the robot based on the action var
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		

		################## get state
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			#resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		for arm in xrange(5):
			data = None
			arm_to_check = '/jaco/jaco_arm_{}_joint_position_controller/'.format(arm)
			print 'checking', arm_to_check
			while data is not None:
				try:
					data = rospy.wait_for_message(arm_to_check, JointControllerState, timeout=5).command
				except:
					pass
			print data
			exit()
			state.append(data)

		for finger in xrange(0,5,2):
			data = None
			finger_to_check = '/jaco/jaco_arm_{}_joint_position_controller/'.format(finger)
			while data is not None:
				try:
					data = rospy.wait_for_message(finger_to_check, JointControllerState, timeout=5).command
				except:
					pass

			state.append(data)

		state = []
		return state

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

if __name__ == '__main__':
	try:
		t = JacoEnv()
	except KeyboardInterrupt as e:
		t._close()
