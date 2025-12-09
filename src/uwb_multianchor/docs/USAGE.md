# UWB MultiAnchor 사용 가이드

## 목차

1. [기본 사용법](#기본-사용법)
2. [고급 설정](#고급-설정)
3. [파라미터 튜닝](#파라미터-튜닝)
4. [예제 시나리오](#예제-시나리오)

## 기본 사용법

### 단일 로봇 시뮬레이션

가장 간단한 사용 방법입니다:

```bash
# 1. 시뮬레이션 시작
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py

# 2. (새 터미널) 로컬라이제이션 시작
ros2 launch turtlebot4_navigation localization.launch.py \
  params_file:=$(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/config/localization_multianchor.yaml

# 3. (새 터미널) Rviz 실행
rviz2 -d $(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/rviz/nav2.rviz
```

### 다른 월드 사용

```bash
# Depot 월드
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py world:=depot

# Maze 월드
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py world:=maze

# Warehouse 월드
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py world:=warehouse
```

### TurtleBot4 Lite 모델 사용

```bash
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py \
  model:=lite \
  world:=depot
```

### 로봇 초기 위치 설정

```bash
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py \
  x:=2.0 \
  y:=3.0 \
  z:=0.1 \
  yaw:=1.57
```

## 고급 설정

### 1. 앵커 위치 커스터마이징

실제 환경에 맞게 앵커 위치를 변경하려면:

#### Step 1: URDF 수정

`urdf/multianchor_sensor.urdf.xacro` 파일의 `<anchors>` 섹션 수정:

```xml
<anchors>
  <!-- 예: 4x4m 방에 4개 앵커 배치 -->
  <anchor xyz="0.0  0.0  3.0" />   <!-- 모서리 1 -->
  <anchor xyz="4.0  0.0  3.0" />   <!-- 모서리 2 -->
  <anchor xyz="4.0  4.0  3.0" />   <!-- 모서리 3 -->
  <anchor xyz="0.0  4.0  3.0" />   <!-- 모서리 4 -->
</anchors>
```

그리고 `<samples>` 값을 앵커 개수에 맞게 변경:

```xml
<horizontal>
  <samples>4</samples>  <!-- 앵커 개수 -->
  ...
</horizontal>
```

#### Step 2: AMCL 설정 수정

`config/localization_multianchor.yaml` 파일 수정:

```yaml
anchors: [
  0.0, 0.0, 3.0,
  4.0, 0.0, 3.0,
  4.0, 4.0, 3.0,
  0.0, 4.0, 3.0
]
```

#### Step 3: 재빌드

```bash
cd ~/ros2_ws
colcon build --packages-select uwb_multianchor
source install/setup.bash
```

### 2. 노이즈 레벨 조정

더 정확하거나 덜 정확한 센서를 시뮬레이션하려면:

#### 거리 측정 노이즈 (URDF)

```xml
<noise>
  <target>range</target>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.1</stddev>  <!-- 0.2 → 0.1: 더 정확 -->
</noise>
```

#### 방위각 측정 노이즈 (URDF)

```xml
<noise>
  <target>azimuth</target>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.0436</stddev>  <!-- 0.087 → 0.0436: 5도 → 2.5도 -->
</noise>
```

#### AMCL 파라미터도 함께 변경

```yaml
sigma_hit: 0.1        # URDF와 일치
sigma_azimuth: 0.0436 # URDF와 일치
```

### 3. 다중 로봇 시뮬레이션

#### 로봇 1 스폰

```bash
ros2 launch uwb_multianchor turtlebot4_multianchor_spawn.launch.py \
  namespace:=robot1 \
  robot_name:=robot1 \
  x:=0.0 \
  y:=0.0
```

#### 로봇 2 스폰

```bash
ros2 launch uwb_multianchor turtlebot4_multianchor_spawn.launch.py \
  namespace:=robot2 \
  robot_name:=robot2 \
  x:=2.0 \
  y:=2.0
```

## 파라미터 튜닝

### AMCL 파티클 필터

#### 더 정확한 위치 추정 (느림)

```yaml
max_particles: 5000   # 2000 → 5000
min_particles: 1000   # 500 → 1000
pf_err: 0.02          # 0.05 → 0.02
```

#### 더 빠른 처리 (덜 정확)

```yaml
max_particles: 1000   # 2000 → 1000
min_particles: 200    # 500 → 200
pf_err: 0.1           # 0.05 → 0.1
```

### 업데이트 빈도

#### 더 자주 업데이트 (높은 CPU 사용)

```yaml
update_min_d: 0.1     # 0.25 → 0.1 (10cm마다)
update_min_a: 0.1     # 0.2 → 0.1 (약 6도마다)
```

#### 덜 자주 업데이트 (낮은 CPU 사용)

```yaml
update_min_d: 0.5     # 0.25 → 0.5 (50cm마다)
update_min_a: 0.5     # 0.2 → 0.5 (약 30도마다)
```

## 예제 시나리오

### 시나리오 1: 창고 환경 탐색

```bash
# 1. 창고 환경 시작
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py \
  world:=warehouse \
  x:=0.0 \
  y:=0.0

# 2. 맵 로드 및 로컬라이제이션
ros2 launch turtlebot4_navigation localization.launch.py \
  map:=$(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/maps/warehouse.yaml \
  params_file:=$(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/config/localization_multianchor.yaml

# 3. Navigation 시작
ros2 launch turtlebot4_navigation nav2.launch.py
```

### 시나리오 2: 커스텀 앵커 배치 테스트

1. 앵커 위치를 실험적으로 배치:
```xml
<!-- 8개 앵커를 원형으로 배치 (반지름 10m, 높이 5m) -->
<anchor xyz="10.0   0.0  5.0" />
<anchor xyz=" 7.07  7.07 5.0" />
<anchor xyz=" 0.0  10.0  5.0" />
<anchor xyz="-7.07  7.07 5.0" />
<anchor xyz="-10.0  0.0  5.0" />
<anchor xyz="-7.07 -7.07 5.0" />
<anchor xyz=" 0.0 -10.0  5.0" />
<anchor xyz=" 7.07 -7.07 5.0" />
```

2. 정확도 테스트:
```bash
# 로봇을 원 안에서 이동시키며 위치 추정 정확도 평가
ros2 topic echo /amcl_pose
```

### 시나리오 3: 센서 융합 (LiDAR + UWB)

아직 구현되지 않았지만, 향후 다음과 같이 사용 가능:

```yaml
# 미래 설정 예시
laser_model_type: "Hybrid"
lidar_weight: 0.5
uwb_weight: 0.5
```

## 데이터 로깅

### 센서 데이터 기록

```bash
# MultiAnchor 스캔 데이터 기록
ros2 bag record /scan -o multianchor_data

# 위치 추정 결과 기록
ros2 bag record /amcl_pose /tf /tf_static -o localization_data
```

### 데이터 재생

```bash
ros2 bag play multianchor_data
```

## 성능 모니터링

### CPU 사용량 확인

```bash
# AMCL 노드 CPU 사용량
top -p $(pgrep -f amcl)
```

### 토픽 주파수 확인

```bash
# 스캔 주파수 (10 Hz 예상)
ros2 topic hz /scan

# AMCL pose 업데이트 주파수
ros2 topic hz /amcl_pose
```

### 파티클 수 모니터링

```bash
ros2 topic echo /particle_cloud | grep -c "x:"
```

## 문제 해결 팁

### 위치 추정이 불안정할 때

1. 파티클 수 증가
2. 업데이트 빈도 증가
3. 앵커 배치 개선 (GDOP 최소화)
4. 노이즈 파라미터 재확인

### 성능이 느릴 때

1. 파티클 수 감소
2. 업데이트 빈도 감소
3. 빔 개수 제한 (`max_beams` 감소)

### 센서 데이터가 이상할 때

1. Gazebo 플러그인 로드 확인
2. TF 트리 확인
3. 센서 visualize 활성화: `<visualize>1</visualize>`
