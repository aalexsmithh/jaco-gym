import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Turtlebot envs
register(
    id='JacoArm-v0',
    entry_point='jaco_gym.envs.jaco_arm:JacoEnv',
    # More arguments here
)