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

class JacoEnv(gazebo_env.GazeboEnv):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		# run the jaco-arm launch file
		gazebo_env.GazeboEnv.__init__(self, "JacoArmWithBallAndCup_v0.launch")

		# self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

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
		return 0

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
