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
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties

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

		self.pubs = [self.joint_0_pub, self.joint_1_pub, self.joint_2_pub, self.joint_3_pub, self.joint_4_pub, self.joint_5_pub, self.finger_0_pub, self.finger_2_pub, self.finger_4_pub]

		# state space
		self.joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.link_states = None
		self.state_sub = rospy.Subscriber('/jaco/joint_states', JointState, self.state_cb)
		self.gz_states = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gz_state_cb)

		rospy.init_node('gym_gazebo')

		# # gym init settings
		self.max_joint_pos = np.asarray([math.pi] * 9)
		self.min_joint_pos = np.asarray([-math.pi] * 9)
		self.action_space = spaces.Box(low=self.min_joint_pos, high=self.max_joint_pos) #-pi to pi for each joint
		self.observation_space = spaces.Box(low=self.min_joint_pos, high=self.max_joint_pos)
		self.reward_range = (1000, 0)
		self.goal = [1,1,1] #[0.167840578046, 0.297489331432, 0.857454500127]
		self.reward = 0.0

		# # gazebo init commands
		self.gazebo_step_size = long(200)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

		# self.hold_init_robot_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5])


	def _step(self, action):
		# rospy.wait_for_service('/gazebo/unpause_physics')
		# try:
		# 	self.unpause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/unpause_physics service call failed")
		######### DO WORK
		done = False

		for i,pub in enumerate(self.pubs):
			pub.publish(Float64(action[i]))

		self.calc_reward()
		tests = 0
		while not self.reached_pos(action, 0.5):
			tests += 1
			time.sleep(0.001)
			if tests > 500:
				done = True
				break

		######### END OF WORK
		# rospy.wait_for_service('/gazebo/pause_physics')
		# try:
		# 	#resp_pause = pause.call()
		# 	self.pause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/pause_physics service call failed")

		### check if the task is done
		
		if self.reward >= 1000:
			done = True
		#check self collision physics??

		return self.joint_state, self.reward, done, {}

	def _reset(self):
		# Resets the state of the environment and returns an initial observation.
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			#reset_proxy.call()
			self.reset_proxy()
			self.hold_init_robot_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_simulation service call failed")

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

	def gz_state_cb(self, msg):
		i = msg.name.index('jaco_on_table::jaco_8_finger_pinkie')
		self.link_states = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]

	def calc_reward(self):
		if self.goal is not None:
			# MSE of link_state from goal
			rwrd = [(x-y)**2 for x,y in zip(self.goal, self.link_states)]
			self.reward = 1/(sum(rwrd)/3)
			if self.reward > 1000:
				self.reward = 1000

	def set_goal(self, goal):
		self.goal = goal

	def reached_pos(self, action, confidence):
		# compare self.joint_states to action from step
		comp = []
		for i,joint in enumerate(self.joint_state):
			comp.append(action[i]-joint)
		if sum(comp) < confidence:
			return True
		else:
			return False

	def hold_init_robot_pos(self, action):
		# rospy.wait_for_service('/gazebo/unpause_physics')
		# try:
		# 	self.unpause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/unpause_physics service call failed")
		
		for i,pub in enumerate(self.pubs):
			pub.publish(Float64(action[i]))

		# rospy.wait_for_service('/gazebo/pause_physics')
		# try:
		# 	self.pause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/pause_physics service call failed")


	def set_physics_update(self, max_dt, hz):
		try:
			phys_getter = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
			phys = phys_getter()
			phys_update = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
			phys_update(time_step=float(max_dt), max_update_rate=float(hz), gravity=phys.gravity, ode_config=phys.ode_config)
		except (rospy.ServiceException) as e:
			print "failed to set physics"

if __name__ == '__main__':
	try:
		t = JacoEnv()
	except KeyboardInterrupt as e:
		t._close()
