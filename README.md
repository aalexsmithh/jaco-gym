jaco-gym
========

This is a WIP package that will allow the Jaco arm Gazebo simulation to be incorporated into OpenAI Gym.

Running
-------
```
$ roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch load_ros_controllers:=true load_velocity_controller:=false
$ python publisher.py
```

Requirements
-----------
This package has a number of dependencies, and the setup is non-trivial

 - [gym-gazebo](https://github.com/erlerobot/gym-gazebo)
	 - gym
	 - Gazebo 8
 - ROS Kinetic
 - [jaco-arm-pkgs](https://github.com/JenniferBuehler/jaco-arm-pkgs)

Sources
-------
 - Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo [1608.05742](https://arxiv.org/abs/1608.05742)

 To look at 
  - https://github.com/pat-coady/trpo/blob/master/notebooks/ant.ipynb
  - https://github.com/wojzaremba/trpo/blob/master/main.py
  - https://github.com/pat-coady/trpo/blob/master/src/train.py
  - https://learningai.io/projects/2017/07/28/ai-gym-workout.html