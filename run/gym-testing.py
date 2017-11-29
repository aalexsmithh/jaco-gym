import gym
import gym_gazebo
import jaco_gym
import time
import numpy
import random
import time


if __name__ == '__main__':
	env = gym.make('JacoArm-v0')
	print "launching the world..."
	raw_input("hit enter when gazebo is loaded...")




	total_episodes = 100
	for x in range(total_episodes):
		action = [1,1,1,1,1,1,1,1,1,1]
		state = env.step(action)
		print state


	# env.close()
