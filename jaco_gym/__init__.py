import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='JacoArm-v0',
    entry_point='jaco_gym.envs.jaco_env:JacoEnv',
    # More arguments here
)