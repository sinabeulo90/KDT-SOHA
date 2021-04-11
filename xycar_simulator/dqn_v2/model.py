# -*- coding: utf-8 -*-

import torch
import torch.nn as nn
import torch.nn.functional as F


# 딥러닝 모델
# - 일반 네트워크와 타겟 네트워크 2가지 모델 이용
class LinearModel(nn.Module):

    def __init__(self, input_size, stack_frame, action_size):   
        super(LinearModel, self).__init__()

        self._input_size = input_size
        self._stack_frame = stack_frame
        self._action_size = action_size

        # 네트워크 모델: 3층의 은닉층으로 구성된 인공신경망
        self.fc1 = nn.Linear(self._input_size*self._stack_frame, 128)
        self.fc2 = nn.Linear(128, 512)
        self.fc3 = nn.Linear(512, 512)
        self.fc4 = nn.Linear(512, self._action_size)
    

    def forward(self, x):
        # 입력: 상태
        # 출력: 각 행동에 대한 Q 값
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x
