#!/usr/bin/env python

from model import *
from dqn import *
from filter import *


def rint(point):
    return np.rint(point).astype(np.int32)


class DQN:
    angle = 0
    speed = 0
    
    def __init__(self):

        self.skip_frame = 4
        self.stack_frame = 10

        model = NN(5, self.stack_frame, 3)
        target_model = NN(5, self.stack_frame, 3)

        self.agent = DQNAgent(model, target_model)
        self.agent.model_load(1110)

        self.filter = Filter(f_cut=5000, freq=20000, num_data=5)


    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_data(self, data):
        new_data = rint(np.clip(self.filter.LPF(np.array(data)), 0, 200))
        print(data, new_data)
        if len(self.agent.observation_set) <= self.skip_frame * self.stack_frame:
            for i in range(self.skip_frame * self.stack_frame):
                self.agent.observation_set.append(new_data)

        state = self.agent.skip_stack_frame(new_data)

        action = self.agent.get_action(state)

        if action == 0:
            self.angle = -50
        elif action == 1:
            self.angle = 0
        else:
            self.angle = 50
        self.speed = 20


    def get_motor(self):
        return self.angle, self.speed

