jaco-gym
========

This is a WIP package that will allow the Jaco arm Gazebo simulation to be incorporated into OpenAI Gym. 

The video below shows an example of a TRPO agent playing cup-and-ball after about 5000 episodes.

![Jaco Arm playing cup-and-ball](https://media.giphy.com/media/kKHA2FnzEG0iuQC30g/giphy.gif)


Running
-------
```
$ cd run
$ python gym-testing.py
```

Requirements
-----------
This package has a number of dependencies, and the setup is non-trivial

 - [gym-gazebo](https://github.com/erlerobot/gym-gazebo)
	 - gym
	 - Gazebo 8
 - ROS Kinetic
 - [jaco-arm-pkgs](https://github.com/JenniferBuehler/jaco-arm-pkgs)
 - [tensorforce](https://github.com/reinforceio/tensorforce)
 - [cup_and_ball](https://github.com/aalexsmithh/cup_and_ball) 

Sources
-------
 - Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo [1608.05742](https://arxiv.org/abs/1608.05742)

 To look at 
  - https://github.com/pat-coady/trpo/blob/master/notebooks/ant.ipynb
  - https://github.com/wojzaremba/trpo/blob/master/main.py
  - https://github.com/pat-coady/trpo/blob/master/src/train.py
  - https://learningai.io/projects/2017/07/28/ai-gym-workout.html
