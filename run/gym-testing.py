import gym
import gym_gazebo
import jaco_gym
import time
import numpy
import random
import time
import json

from tensorforce.agents import Agent
from tensorforce.execution import Runner
from tensorforce.contrib.openai_gym import OpenAIGym
from tensorforce.agents.dqn_agent import DQNAgent


network_spec = [
	dict(type='dense', size=32, activation='relu'),
	dict(type='dense', size=32, activation='relu')
]


def main():
	#tensorforce
	env = OpenAIGym('JacoArm-v0')

	agent = agent = DQNAgent(
    	states_spec=env.states,
    	actions_spec=env.actions,
    	network_spec=network_spec,

	runner = Runner(agent=agent, environment=env)

	raw_input("hit enter when gazebo is loaded...")
	runner.run(episodes=100, max_episode_timesteps=100)

	# #old-fashioned way
	# env = gym.make('JacoArm-v0')
	# print "launching the world..."
	# #gz loaing issues, let user start the learning
	# raw_input("hit enter when gazebo is loaded...")

	# # env.set_goal([0.167840578046, 0.297489331432, 0.857454500127])

	# total_episodes = 100
	# action = [1,1,1,1,1,1,1,1,1,1]
	# x = 0
	# # for x in range(total_episodes):
	# while True:
	# 	# if x % 10 is 0:
	# 	action = numpy.random.rand(1, 10)[0]
	# 		# print 'new action is', action
		
	# 	state, reward, done, _ = env.step(action)
	# 	print reward
	# 	time.sleep(0.2)
	# 	x += 1


	env.close()


if __name__ == '__main__':
	main()