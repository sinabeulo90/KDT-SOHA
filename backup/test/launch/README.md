# 기능 상세 설명

```diff
@@ Launch 코드에 대한 초안입니다. 수정사항이나 추가했으면 하는 부분, 구조상 불편한 부분이 있으면 알려주세요. @@
```



## 실행 방법


1. 기본적인 실행

    ```bash
    $ roslaunch soha_drive main_b2.launch
    ```

2. ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **`rosbag record` 기능을 켜고 실행**

    ```bash
    $ roslaunch soha_drive main_b2.launch record:=true
    ```

3. 테스트 실행

    ```bash
    $ roslaunch soha_drive test.launch
    ```

> 2번 항목처럼 record 기능을 켜고 끄는 기능처럼 `play:=path`와 같이 시도했지만, 자동완성 기능이 지원되지 않아 구현하지 않았습니다.



## 디렉토리 구조

```bash
launch/
├── main_a2.launch
├── main_b2.launch
├── test.launch
└── xycar
    ├── camera.launch           # [v] 카메라
    ├── camera_test.launch      # [ ] 카메라 테스트
    ├── lidar.launch            # [ ] 라이다
    ├── rviz.launch             # [ ] RVIZ
    ├── ultrasonic.launch       # [ ] 초음파
    ├── xycar_motor_a2.launch   # [ ] Xycar A2 모터 제어
    └── xycar_motor_b2.launch   # [v] Xycar B2 모터 제어
```

`./launch/xycar` 폴더를 제외하고 자유롭게 사용하면 됩니다. xycar 폴더 안에 있는 파일은 Xycar 장비에 장착되어 있는 센서들을 사용하는데 필요한 노드을 구분하여 launch파일로 만들었습니다.  
[v] 표시는 정상 동작을 확인했다는 의미이고, [ ] 표시는 아직 정상 동작을 확인하지 못했다는 의미입니다. 혹시 사용하시다가 정상 동작이 확인되면 업데이트 부탁드립니다.



### 수정 가능 파일

1. main_a2.launch, main_b2.launch

과제를 수행하기 위해 필요한 센서를 추가/해제할 때, 또는 임의의 노드들을 추가하기 위해 사용할 수 있습니다.  
예를 들어, 과제에서 카메라 센서 대신 초음파 센서를 사용한다면, 아래와 같이 주석을 추가/해제하여 바꿀 수 있습니다.

```XML
<!-- <include file="$(find soha_drive)/launch/xycar/camera.launch" /> -->
<include file="$(find soha_drive)/xycar/launch/ultrasonic.launch" />

```

2. 임의의 launch 파일 추가

`./launch` 폴더 내애서 launch 파일을 추가하고 수정해주시면 됩니다. 1번 파일을 참고하면서 사용하세요.



### ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) 수정 금지 파일: `launch/xycar`

`launch/xycar`에 있는 launch 파일은 Xycar 하드웨어에 장착된 센서 및 모터 제어 관련 노드들이기 때문에 직접 수정하지 않습니다. 
