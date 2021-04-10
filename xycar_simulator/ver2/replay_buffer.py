# -*- coding: utf-8 -*-

import random
import numpy as np
from collections import deque


class ReplayBuffer():

    def __init__(self, stack_frame, memory_maxlen=int(1e+4)):
        self._stack_frame = stack_frame

        self._buffer = deque(maxlen=memory_maxlen)
        self._stack_buffer = deque(maxlen=stack_frame)
    
    
    def put(self, data):
        self._buffer.append(data)
    

    def get_state(self, obs):
        while len(self._stack_buffer) < self._stack_frame:
            self._stack_buffer.append(obs)
        self._stack_buffer.append(obs)
        
        state = np.stack(self._stack_buffer, axis=0).flatten()
        return state


    def sample(self, batch_size):
        return random.sample(self._buffer, batch_size)
    

    def size(self):
        return len(self._buffer)


    def reset(self):
        self._stack_buffer.clear()
