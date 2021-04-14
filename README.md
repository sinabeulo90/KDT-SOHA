# [K-Digital Training: 프로그래머스 자율주행 데브코스](https://programmers.co.kr/learn/courses/10822)

실제 자동차의 1/10의 모형 차인 자이카를 통해 OpenCV 영상처리 기술과 머신 러닝 기술을 융합시켜 자율 주행 SW를 직접 설계해 구현한 내용들 중 자율주행 경진대회에서 사용했던 코드들을 보관한 저장소입니다.

모의 자율주행 경진대회는 총 3차까지 진행되었고, 업로드한 코드들은 2차와 3차와 관련된 것들만 업로드 하였습니다.

Notion 참고: [소하(Software & Hardware) 팀](https://www.notion.so/Software-Hardware-1a1cd7284e5d47f8bad64bfcd0db77f3)


<br>


## 1차 모의 자율주행 경진대회

1차 경진대회의 목표는 영상처리를 이용한 차선 인식 주행인데, 이는 3차 경진대회에서 포함되는 기술이기 때문에 따로 포함하지 않았습니다.

Notion 참고: [첫번째 월간 프로젝트 1차 모의 자율주행 경진대회 기록](https://www.notion.so/1-13e53617481f4baf83dfd3e7d4f52caf)


<br>


## 2차 모의 자율주행 경진대회

2차 모의 자율주행 경진대회는 **장애물 회피 및 주차**가 목표였고, 다루었던 센서와 기술들은 아래와 같습니다.

1. 초음파 센서 주행
2. QR 코드 인식에 따른 미션 구분
    1. 전진 방향을 후진 방향으로 전환
    2. 강화학습 주행
    3. YOLO 인식
3. AR Tag를 이용한 주차

이 중 담당했던 부분은 **2.2 강화학습 주행**이며, [xycar_simulator](https://github.com/sinabeulo90/KDT-SOHA/tree/main/xycar_simulator)에 있습니다.

2차 경진대회에서 강화학습 주행은 미션을 완수하지 못했습니다다. 대회가 끝난 뒤에, 약간의 학습 과정을 수정하고 나서, 시뮬레이션으로 이전보다 훨씬 높은 성능의 주행을 할 수 있음을 확인할 수 있었습니다.

Notion 참고: [두번째 월간 프로젝트 2차 모의 자율주행 경진대회 기록](https://www.notion.so/2-920963504e8a4e5c8ac666052b3d00b4)


<br>


## 3차 모의 자율주행 경진대회

3차 모의 자율주행 경진대회도 **장애물 회피 및 주차**가 목표였고, 다루었던 센서와 기술들은 아래와 같습니다.

|  | 차선 인식 주행 | 장애물 회피 | 횡단보도 정차 | 주차 |
|:-:|:-:|:-:|:-:|:-:|
| 영상처리 | O | | O | |
| Lidar 센서 | | O | | |
| 초음파 센서 | | | | O | O |
| AR Tag 인식 | | | | O |

이 중 담당했던 부분은 **영상처리를 이용한 차선 인식 주행과 기능 통합**이며, [soha_workspace](https://github.com/sinabeulo90/KDT-SOHA/tree/main/soha_workspace)에 있습니다.

3차 경진대회에서 모든 미션은 완수했지만, 성적은 좋지 않았습니다. 원인으로는 비용이 큰 영상처리 방식 사용으로 인한 낮은 fps와 일반화된 기능 수행이었습니다. 일반화된 기능 수행이란, 차량이 주행하면서 위의 모든 미션을들 확인한다는 점 입니다.

**경쟁팀의 미션 수행이 환경에 의존적이었다면, 저희 팀은 환경에 관계없이 모든 미션을 수행할 수 있도록 구현이 되었다는 점이 가장 큰 차별점이라고 생각합니다.**