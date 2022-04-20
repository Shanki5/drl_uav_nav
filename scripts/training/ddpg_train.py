#! /usr/bin/env python3
import gym
import rospy
import numpy as np
import os
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from gym.envs.registration import register

from stable_baselines3.ddpg.policies import MlpPolicy
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines3 import DDPG

from drl_uav_nav.task_env import parrot_continous_task_env

task_and_robot_environment_name = rospy.get_param(
        '/drone/task_and_robot_environment_name')


reg = register(
    id='Parrotdrone_Continuous_action-v0',
    entry_point='drl_uav_nav.task_env.parrot_continuous_task_env:ParrotDroneGotoContinuous',
    max_episode_steps=100000,
    )

env = gym.make(task_and_robot_environment_name)
#env = gym.make('MountainCarContinuous-v0')

# the noise objects for DDPG
n_actions = env.action_space.shape[-1]
param_noise = None
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

# logs_base_dir = './tensor_logs'
# os.makedirs(logs_base_dir, exist_ok=True)
model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise)
model.learn(total_timesteps=400000)
model.save("ddpg_drone")

#del model # remove to demonstrate saving and loading

model = DDPG.load("ddpg_drone")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
