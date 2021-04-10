# -*- coding: utf-8 -*-

import os
import io
import random

import numpy as np

import torch
import torch.optim as optim
import torch.nn.functional as F


class DQNAgent():

    def __init__(self, model, target_model=None, discount_factor=0.99, learning_rate=1e-4, epsilon=1.0):
        self._device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
        self._model = model.to(self._device)
        self._epsilon = epsilon
        self._epsilon_min = 0.01
        self._discount_factor = discount_factor

        if target_model:
            self._target_model = target_model.to(self._device)
            self.update_target()

        # if torch.cuda.is_available():
        #     self.model = model.cuda()
        #     self.target_model = self.target_model.cuda()
        
        self.optimizer = optim.Adam(self._model.parameters(), lr=learning_rate)
        self.loss = F.mse_loss


    def update_target(self):
        self._target_model.load_state_dict(self._model.state_dict())
    

    def get_action(self, state):
        if self._model.training and np.random.sample() < self._epsilon:
            return np.random.randint(0, self._model._action_size)
        
        with torch.no_grad():
            state = torch.FloatTensor(state).unsqueeze(0).to(self._device)
            q = self._model(state)
            return np.argmax(q.cpu().detach().numpy())


    def epsilon_decay(self, decay=1e-4):
        self._epsilon = max(self._epsilon_min, self._epsilon - decay)


    def train_model(self, replay_buffer, batch_size):
        batches = replay_buffer.sample(batch_size)

        state_batch, action_batch, reward_batch, next_batch, done_batch = [], [], [], [], []

        for _state, _action, _reward, _next, _done in batches:
            state_batch.append(np.stack(_state, axis=0))
            action_batch.append(_action)
            reward_batch.append(_reward)
            next_batch.append(np.stack(_next, axis=0))
            done_batch.append(_done)

        state_batch     = torch.FloatTensor(state_batch).to(self._device)
        action_batch    = torch.LongTensor(action_batch).unsqueeze(1).to(self._device)
        reward_batch    = torch.FloatTensor(reward_batch).unsqueeze(1).to(self._device)
        next_batch      = torch.FloatTensor(next_batch).to(self._device)
        done_batch      = torch.FloatTensor(done_batch).unsqueeze(1).to(self._device)

        q = self._model(state_batch).gather(1, action_batch)

        with torch.no_grad():
            max_q = self._target_model(next_batch).max(1)[0].unsqueeze(1)
            target_q = reward_batch + self._discount_factor * max_q * done_batch

        loss = self.loss(q, target_q).to(self._device)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.cpu().detach().numpy(), max_q.cpu().numpy()


    def save_model(self, episode, comment=None):
        script_dir = os.path.dirname(__file__)

        if comment: 
            dir_path = os.path.join(script_dir, "save_dqn_{}".format(comment))
        else:
            dir_path = os.path.join(script_dir, "save_dqn")

        if not os.path.isdir(dir_path):
            os.mkdir(dir_path)

        save_path = os.path.join(dir_path, "main_model_{:06}.pth".format(episode))

        # Python2
        buffer = io.BytesIO()
        torch.save(self._model.state_dict(), buffer, _use_new_zipfile_serialization=False)
        with open(save_path, "wb") as f:
            f.write(buffer.getvalue())


    def load_model(self, episode, comment= None, eval=True):
        script_dir = os.path.dirname(__file__) 

        if comment: 
            dir_path = os.path.join(script_dir, "save_dqn_{}".format(comment))
        else:
            dir_path = os.path.join(script_dir, "save_dqn")

        load_path = os.path.join(dir_path, "main_model_{:06}.pth".format(episode))
        
        # Python2
        with open(load_path, "rb") as f:
            buffer = io.BytesIO(f.read())
            self._model.load_state_dict(torch.load(buffer, map_location=self._device))
            self._model.to(self._device)

        if eval:
            self._model.eval()
        else:
            self.update_target()
