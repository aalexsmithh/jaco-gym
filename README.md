jaco-gym
========

Reinforcement learning on the Jaco Arm in Gazebo using Gym. An honours project at McGill University.

This is a WIP package that will allow the Jaco arm Gazebo simulation to be incorporated into OpenAI Gym. 

<img width="1566" height="922" alt="frame093" src="https://github.com/user-attachments/assets/f18d09ad-6600-405c-b76f-e79cd28756ce" />


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

Screenshots
-----------

Image 1            |  Image 2
:-------------------------:|:-------------------------:
<img src="https://github.com/user-attachments/assets/e98670a7-c73a-4c0f-ad18-8615b54a52b3" width="350">  |  <img src="https://github.com/user-attachments/assets/1b85c4f7-f5cf-432e-904f-0a680f4d7f37" width="350">

