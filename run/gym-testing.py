import gym
import gym_gazebo
import jaco_gym
import time
import numpy
import random
import time


if __name__ == '__main__':
	env = gym.make('JacoArm-v0')

	total_episodes = 100
	for x in range(total_episodes):
		action = [1,1,1,1,1,1,1,1,1,1]
		env.step(action)

	env.close()
