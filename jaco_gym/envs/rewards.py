'''
this contains all the reward functions tested for this project. 
new functions can be added expecting the environment itself passed as a variable to the function.
'''

def reward6(env):
	rwrd = [(x-y)**2 for x,y in zip(env.cup_state, env.ball_state)]
	# print 'cup state:', env.cup_state, "\tball state:", env.ball_state, '\t rwrd:', rwrd
	env.reward = 3/sum(rwrd)

	# check if below lip of the cup
	if env.cup_state[2] - env.ball_state[2] > 0:
		env.reward /= 100

	# check if cup has dropped
	if env.cup_state[2] < 0.5:
		env.reward = 0

	rwrd = [(x-y)**2 for x,y in zip(env.cup_state, env.pinky_state)]
	env.reward -= (sum(rwrd)/3)

def reward7(env):
	if env.cup_state[2] - env.ball_state[2] > 0:
		env.reward = 0
	else: # mse^-1 above the cup
		rwrd = [(x-y)**2 for x,y in zip(env.cup_state, env.ball_state)]
		env.reward = 3/sum(rwrd)

	#check mse cup hand
	rwrd = [(x-y)**2 for x,y in zip(env.cup_state, env.pinky_state)]
	env.reward -= (sum(rwrd)/3)

def calc_reward(env):
	if env.goal is not None:
		# MSE of link_state from goal
		rwrd = [(x-y)**2 for x,y in zip(env.goal, env.link_states)]
		env.reward = 1/(sum(rwrd)/3)
		if env.reward > 1000:
			env.reward = 1000

def calc_reward_cupball(env):
	rwrd = [(x-y)**2 for x,y in zip(env.cup_state, env.ball_state)]
	# print 'cup state:', env.cup_state, "\tball state:", env.ball_state, '\t rwrd:', rwrd
	env.reward = 3/sum(rwrd)

	# check if below lip of the cup
	if env.cup_state[2] - env.ball_state[2] > 0:
		env.reward /= 100

	# check if cup has dropped
	if env.cup_state[2] < 0.5:
		env.reward = 0