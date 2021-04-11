# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.getcwd()))

import cv2 as cv
import numpy as np

from dqn_v1.dqn import *
from simulator.simulator import Simulator

np.random.seed(42)

load_model_comment = None
load_model_episode = 1250

sensor_count = 8
action_size = 6

skip_frame = 5
stack_frame = 10

learning_rate = 0.0001
epsilon_init = 0.3
memory_maxlen = 1000000

fps = 5

model = LinearModel(sensor_count, stack_frame, action_size)
agent = DQNAgent(model, learning_rate, epsilon_init, skip_frame, stack_frame, memory_maxlen)
agent.model_load(load_model_episode)

env = Simulator(map="rally_map.png", fps=fps)

if not os.path.isdir("video"):
    os.mkdir("video")

background = env.render(display=False)
height, width = background.shape[:2]

episode = 0

while True:
    gear_drive_cnt = 0
    gear_reverse_cnt = 0
    
    obs, _, _ = env.reset()

    agent.reset(obs)
    gear = env.BREAK

    state = agent.skip_stack_frame(obs)

    filename = "video/{:06d}.avi".format(load_model_episode)
    
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    writer = cv.VideoWriter(filename, fourcc, fps, (width, height))

    while not env.is_done:
        background = env.render(fps=fps, display=False)

        writer.write(background)

        action = agent.get_action(state)

        # 조향각 조정
        if action % 3 == 0:
            steering_deg = -env.max_steering_deg
        elif action % 3 == 1:
            steering_deg = 0
        elif action % 3 == 2:
            steering_deg = env.max_steering_deg

        # 기어 조정
        if action // 3 == 0:
            if gear != env.DRIVE:
                env.car.reset()
            gear = env.DRIVE
            gear_drive_cnt += 1
        elif action // 3 == 1:
            if gear != env.REVERSE:
                env.car.reset()
            gear = env.REVERSE
            gear_reverse_cnt += 1

        next_obs, _, _ = env.step(gear, steering_deg)
        next_state = agent.skip_stack_frame(next_obs)

        state = next_state

    episode += 1

    print("Episode {: >10d} | Drive/Reverse: {}/{}".format(episode, gear_drive_cnt, gear_reverse_cnt))

    writer.release()
    break