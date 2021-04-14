# Xycar Simulator


![Python 3.9](https://img.shields.io/badge/Python-3.9-blue?logo=Python)

![DQN을 적용한 초음파 센서 장애물 회피 주행](/assets/xycar_simulator/dqn.gif)

[YouTube 링크](https://youtu.be/GPqNeLPR5EA)


<br>


## 1. 대회 결과
결론적으로는 DQN을 적용한 장애물 회피 주행을 성공하지 못했다. 약 2시간 동안 약3500번의 에피소드를 학습한 모델을 적용했기 때문에, 학습 시간의 문제라고 생각했다.

그래서 대회를 마치고 기존 모델로 24시간 학습을 진행한 뒤 시뮬레이터로 확인했지만, 이전 주행 결과와 유사하게 전/후진을 반복하면서 주행하지 못했다. 아래 loss 그래프를 보면 거의 0으로 수렴하고 있기 떄문에, environment에서 reward를 잘못 주고 있거나, agent에게 전달되는 feature의 구성이 잘못되었건, 정보가 부족하다고 생각했다.

![DQN v2 Simulation](/assets/xycar_simulator/dqn_simulation_v1.gif)


[YouTube 링크](https://youtu.be/y8u4eq6At2Q)

| 기존 모델의 Loss |
|:-:|
| ![DQN v1 Loss](/assets/xycar_simulator/dqn_loss_v1.png) |


<br>


## 2. 개선 과정 및 결과
따라서 아래와 같이 수정을 한 뒤 24시간 학습을 진행하였고, 결과는 다음과 같다.

| | 수정 전 | 수정 후 |
|:-:|:-|:-|
| Feature 구성 | Agent가 어떤 action을 선택한 뒤 차량이 1-Step 움직이면, 가장 최근 feature 부터 5개씩 skip된 10개의 feature를 학습에 사용 | Agent가 어떤 action을 선택한 뒤 차량이 **5-Step**움직이면, 가장 최근 10개의 feature를 학습에 사용 |
| Reward | +1: 전진할 경우 <br> -1: 후진할 경우, 장애물을 부딪힐 경우 <br> 0: 정지할 경우 | **차량의 현재 위치를 기준으로 reward 도메인을 설정** <br> +1: 도메인을 통과할 때, 속도가 +일 경우 <br> -1: 도메인을 통과할 때, 속도가 -일 경우 |
| Batch Size | 1024 | 32 |
| Epsilon | 0.3 | 1.0 |
| Epsilon Decay | 0.00001 <br> 매 step마다 적용 | 0.0001 <br> 학습이 시작되고, 매 episode 마다 적용 |
| Discount Factor | 0.9 | 0.99 |
| 학습을 시작하는 Memory Buffer 최소 길이 | 5120 | 1000 |
| Target Network 복사 | 매 50 episode | 매 1000 episode |

![DQN v2 Simulation](/assets/xycar_simulator/dqn_simulation_v2.gif)

[YouTube 링크](https://youtu.be/jhu-p38IB2k)

가장 큰 특징은 agent에 들어가는 시계열 feature를 memory buffer에서 skip하면서 구성하지 않고, environment에서 미리 skip 횟수만큼 업데이트 한 뒤, 현재 차량 위치에서 얻은 feature를 memory buffer에 저장하고, memory buffer에서 시계열 feature를 구성했다는 점이다.

기존 모델로 학습할 경우, agent의 초기 Q-value는 무작위 행동을 하게 되므로 전/후진을 반복하게 되는데, 이 상태에서 구성되는 시계열 feature들은 대부분 초기에 머무는 위치를 벗어나지 못한다. 시계열 feature라는 것을 좀 더 명확히 하기 위해 처음부터 agent가 선택한 행동에 대해 충분히 움직인 뒤에 feature들을 memory buffer에 추가하였다.

여러가지 하이퍼파라미터들이 변경되기는 했지만, 이 부분들이 주행에 큰 역할을 한다고는 생각하지 않았다. 대회 당시 기존 모델로 여러가지 하이퍼파라미터를 조절하면서 2-3시간씩 학습했을 때, 시뮬레이션 상으로 주행 형태가 거의 비슷한 꼴을 보였기 때문에, 하이퍼파라미터만으로는 극적인 결과를 바라는 건 어려울 것이라고 생각한다.

다만, 학습 결과가 전체적으로 목표로 했던 바에 일치하는지 확인이 된 뒤에는, 좀 더 나은 성능 개선을 위해 하이퍼파라미터 조정이 필요하다고 생각한다. 기존의 방식에서는 agent가 주어진 feature가 시계열 데이터라는 것을 인지하지 못한 상태에서 학습을 진행했고, 올바르게 움직이지 않는 상태에서 하이퍼파라미터만 조정했기 때문에 결과는 항상 비슷했던 것 아닐까 생각하기 때문이다.

| Loss | Max Q | Reward |
|:-:|:-:|:-:|
| ![DQN Loss](/assets/xycar_simulator/dqn_loss.png) | ![DQN Max Q](/assets/xycar_simulator/dqn_max_q.png) | ![DQN Reward](/assets/xycar_simulator/dqn_reward.png) |
| 수정 전(파랑) / 수정 후(주황) | 수정 전(파랑) / 수정 후(주황) | 수정 전(파랑) / 수정 후(주황) |


<br>


## 3. 폴더 구조

```text
Xycar_simulator
├── dqn_v1      # 2차 경진대회 당시 사용한 DQN
│
├── dqn_v2      # 수정 후 DQN
│
└── simulator   # 시뮬레이터 모듈
    │   
    ├── car.py          # 차량 정보
    │
    ├── map             # 지도 이미지
    │   ├── rally_map.png
    │   ├── rally_map2.png
    │   └── rally_map3.png
    │
    ├── simulator.py    # 차량 정보와 지도 정보를 사용하여,
    │                   # 시뮬레이션을 수행하는 시뮬레이터 클래스
    │
    └── utils.py        # 차량 정보와 지도 정보를 사용하여,
                        # 시뮬레이터에 필요한 정보를 계산하는 유틸
```
