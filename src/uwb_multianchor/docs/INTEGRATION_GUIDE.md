# UWB MultiAnchor 로봇 통합 가이드

## 목차
1. [개요](#개요)
2. [센서 매크로 사용법](#센서-매크로-사용법)
3. [단계별 통합 가이드](#단계별-통합-가이드)
4. [AMCL 설정](#amcl-설정)
5. [Launch 파일 작성](#launch-파일-작성)
6. [예제: TurtleBot4](#예제-turtlebot4)
7. [예제: 커스텀 로봇](#예제-커스텀-로봇)
8. [트러블슈팅](#트러블슈팅)

---

## 개요

`uwb_multianchor` 패키지는 **로봇 플랫폼에 독립적인** UWB 기반 측위 센서를 제공합니다. 어떤 로봇에도 쉽게 추가할 수 있도록 설계되었습니다.

### 필요한 것

1. **로봇 URDF**: 센서를 부착할 로봇 모델
2. **Gazebo Harmonic**: MultiAnchor 센서 지원
3. **Navigation2** (선택): AMCL 기반 위치 추정
4. **앵커 좌표**: 환경에 고정된 앵커들의 3D 위치

---

## 센서 매크로 사용법

### 기본 매크로: `uwb_multianchor_sensor`

**위치**: `urdf/multianchor_sensor.urdf.xacro`

**파라미터**:
- `name`: 센서 이름 (예: "uwb_tag", "multianchor")
- `parent_link`: 부모 링크 (센서를 부착할 로봇의 링크)
- `origin`: 부모 링크 기준 센서의 위치 및 방향

**기본 사용법**:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 1. MultiAnchor 센서 매크로 포함 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <!-- 2. 센서 추가 -->
  <xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </xacro:uwb_multianchor_sensor>

</robot>
```

### 센서 설정 커스터마이징

센서 파라미터를 변경하려면 `multianchor_sensor.urdf.xacro` 파일을 복사하여 수정하거나, 매크로에 파라미터를 추가할 수 있습니다.

---

## 단계별 통합 가이드

### Step 1: 패키지 설치

```bash
cd ~/ros2_ws/src
# uwb_multianchor 패키지 복사 또는 클론
colcon build --packages-select uwb_multianchor
source install/setup.bash
```

### Step 2: 로봇 URDF에 센서 추가

**예제: `my_robot.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 기존 로봇 정의 -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot_base.urdf.xacro" />
  
  <!-- UWB MultiAnchor 센서 추가 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <!-- 센서를 base_link에 부착 (로봇 중심 위 10cm) -->
  <xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </xacro:uwb_multianchor_sensor>

</robot>
```

### Step 3: 앵커 좌표 설정

센서 URDF 내부의 앵커 좌표를 환경에 맞게 수정:

**방법 A: 원본 수정**

`urdf/multianchor_sensor.urdf.xacro` 파일에서:

```xml
<anchors>
  <!-- 환경에 맞게 수정 -->
  <anchor xyz="0.0  0.0  3.0" />  <!-- 앵커 1 -->
  <anchor xyz="5.0  0.0  3.0" />  <!-- 앵커 2 -->
  <anchor xyz="5.0  5.0  3.0" />  <!-- 앵커 3 -->
  <anchor xyz="0.0  5.0  3.0" />  <!-- 앵커 4 -->
</anchors>
```

**방법 B: 로봇별 커스텀 센서 파일 생성**

```bash
cp $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/urdf/multianchor_sensor.urdf.xacro \
   ~/ros2_ws/src/my_robot_description/urdf/my_robot_uwb_sensor.urdf.xacro
```

그리고 `my_robot_uwb_sensor.urdf.xacro`에서 앵커 좌표 수정:

```xml
<xacro:include filename="$(find my_robot_description)/urdf/my_robot_uwb_sensor.urdf.xacro" />
```

### Step 4: Launch 파일 작성

**`my_robot_spawn.launch.py`**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_my_robot = FindPackageShare('my_robot_description')
    
    # Robot description
    robot_description = Command([
        'xacro', ' ',
        PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_robot.urdf.xacro']),
        ' gazebo:=gz'
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-topic', 'robot_description',
        ],
        output='screen',
    )
    
    # UWB sensor static TF (센서 링크 → Gazebo 토픽)
    uwb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'uwb_tag_link',
            'my_robot/uwb_tag_link/uwb_tag'
        ]
    )
    
    # ROS-Gazebo bridge for UWB scan
    uwb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uwb_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/model/my_robot/uwb_tag_link/uwb_tag/scan' +
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/model/my_robot/uwb_tag_link/uwb_tag/scan', '/scan')
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
        uwb_tf,
        uwb_bridge,
    ])
```

### Step 5: AMCL 설정 (Navigation2 사용 시)

**`my_robot_localization.yaml`**:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    
    # Frame IDs
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # UWB Tag & Anchor 센서 모델
    laser_model_type: "Tag&Anchors"
    
    # 앵커 좌표 (URDF와 정확히 일치해야 함!)
    anchors: [
      0.0, 0.0, 3.0,   # 앵커 1
      5.0, 0.0, 3.0,   # 앵커 2
      5.0, 5.0, 3.0,   # 앵커 3
      0.0, 5.0, 3.0    # 앵커 4
    ]
    
    # 노이즈 파라미터 (URDF와 일치해야 함!)
    sigma_hit: 0.2        # 거리 측정 표준편차 (m)
    sigma_azimuth: 0.087  # 방위각 측정 표준편차 (rad, 5도)
    
    # 파티클 필터
    max_particles: 2000
    min_particles: 500
    
    # 기타 AMCL 파라미터...
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    update_min_d: 0.25
    update_min_a: 0.2
```

---

## AMCL 설정

### 필수 파라미터

#### 1. `laser_model_type`
```yaml
laser_model_type: "Tag&Anchors"
```
- **필수**: UWB 센서를 사용하려면 반드시 설정

#### 2. `anchors`
```yaml
anchors: [x1, y1, z1, x2, y2, z2, ..., xN, yN, zN]
```
- **형식**: 1차원 배열 `[x, y, z, x, y, z, ...]`
- **단위**: 미터 (m)
- **좌표계**: 맵 프레임 기준 (global)
- **중요**: URDF의 `<anchor>` 태그와 **정확히 일치**해야 함

#### 3. `sigma_hit` (거리 노이즈)
```yaml
sigma_hit: 0.2  # 20cm 표준편차
```
- URDF `<noise target="range">` 의 `<stddev>` 값과 일치

#### 4. `sigma_azimuth` (방위각 노이즈)
```yaml
sigma_azimuth: 0.087  # 5도 (라디안)
```
- URDF `<noise target="azimuth">` 의 `<stddev>` 값과 일치

### 파라미터 튜닝 가이드

| 파라미터 | 값 범위 | 효과 |
|---------|---------|------|
| `sigma_hit` | 0.1 ~ 0.5 | 작을수록 정확, 너무 작으면 발산 |
| `sigma_azimuth` | 0.05 ~ 0.2 | 작을수록 정확, 너무 작으면 발산 |
| `max_particles` | 1000 ~ 5000 | 많을수록 정확하지만 느림 |
| `update_min_d` | 0.1 ~ 0.5 | 업데이트 주기 (이동 거리 기준) |

---

## Launch 파일 작성

### 최소 구성

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Robot State Publisher (URDF 발행)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': '...'}]
    )
    
    # 2. Spawn Robot (Gazebo)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description']
    )
    
    # 3. UWB TF Static Publisher
    uwb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'uwb_tag_link', 'my_robot/uwb_tag_link/uwb_tag']
    )
    
    # 4. ROS-Gazebo Bridge
    uwb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/my_robot/uwb_tag_link/uwb_tag/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/model/my_robot/uwb_tag_link/uwb_tag/scan', '/scan')]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
        uwb_tf,
        uwb_bridge,
    ])
```

### 주의사항

1. **TF 프레임 이름**: 
   - URDF의 센서 링크 이름: `<name>_link` (예: `uwb_tag_link`)
   - Gazebo 토픽: `/model/<robot_name>/<name>_link/<name>/scan`

2. **토픽 리매핑**:
   - AMCL은 `/scan` 토픽을 구독
   - Bridge가 Gazebo 토픽을 `/scan`으로 리매핑

3. **네임스페이스**:
   - 다중 로봇 사용 시 각 로봇에 네임스페이스 부여

---

## 예제: TurtleBot4

전체 예제는 `examples/turtlebot4/` 디렉토리를 참조하세요.

### 파일 구조

```
examples/turtlebot4/
├── urdf/
│   ├── turtlebot4_standard_multianchor.urdf.xacro
│   └── turtlebot4_lite_multianchor.urdf.xacro
├── launch/
│   ├── turtlebot4_multianchor_spawn.launch.py
│   └── turtlebot4_multianchor_sim.launch.py
└── config/
    └── localization_multianchor.yaml
```

### 사용법

```bash
# 시뮬레이션 실행
ros2 launch uwb_multianchor turtlebot4_multianchor_sim.launch.py

# 또는 예제 파일 직접 사용
ros2 launch \
  $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py
```

### URDF 예제 (Standard)

```xml
<?xml version="1.0" ?>
<robot name="turtlebot4" xmlns:xacro="http://ros.org/wiki/xacro">
  
  <!-- 기존 TurtleBot4 포함 -->
  <xacro:include filename="$(find turtlebot4_description)/urdf/standard/turtlebot4.urdf.xacro" />
  
  <!-- UWB 센서 추가 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <!-- 센서 위치 설정 -->
  <xacro:property name="uwb_x" value="0.00393584"/>
  <xacro:property name="uwb_y" value="0.0"/>
  <xacro:property name="uwb_z" value="0.07529272"/>
  
  <!-- 센서 부착 (shell_link에) -->
  <xacro:uwb_multianchor_sensor name="multianchorsensor" parent_link="shell_link">
    <origin xyz="${uwb_x} ${uwb_y} ${uwb_z}" rpy="0 0 ${pi/2}"/>
  </xacro:uwb_multianchor_sensor>

</robot>
```

---

## 예제: 커스텀 로봇

### 시나리오: 간단한 차동구동 로봇

**로봇 사양**:
- 이름: `simple_robot`
- 크기: 0.5m x 0.4m x 0.2m
- 베이스 링크: `base_link`
- UWB 센서 위치: 로봇 중심 위 15cm

### Step 1: 로봇 URDF 생성

**`simple_robot_with_uwb.urdf.xacro`**:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 기본 속성 -->
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- 바퀴들 (생략) -->
  
  <!-- UWB 센서 추가 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
  </xacro:uwb_multianchor_sensor>
  
  <!-- Base footprint -->
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

</robot>
```

### Step 2: 앵커 배치 설계

**환경**: 10m x 10m 방, 천장 높이 3m

**앵커 배치** (모서리 4개):

```
      (0,10,3)           (10,10,3)
         ●─────────────────●
         │                 │
         │                 │
         │    (5,5,0)      │  <- 로봇 시작 위치
         │       R         │
         │                 │
         ●─────────────────●
      (0,0,3)            (10,0,3)
```

### Step 3: 센서 파일 커스터마이징

```bash
mkdir -p ~/ros2_ws/src/simple_robot_description/urdf
cp $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/urdf/multianchor_sensor.urdf.xacro \
   ~/ros2_ws/src/simple_robot_description/urdf/simple_robot_uwb.urdf.xacro
```

**`simple_robot_uwb.urdf.xacro` 수정**:

```xml
<anchors>
  <anchor xyz=" 0.0  0.0 3.0" />  <!-- 좌하 -->
  <anchor xyz="10.0  0.0 3.0" />  <!-- 우하 -->
  <anchor xyz="10.0 10.0 3.0" />  <!-- 우상 -->
  <anchor xyz=" 0.0 10.0 3.0" />  <!-- 좌상 -->
</anchors>

<horizontal>
  <samples>4</samples>  <!-- 8 → 4로 변경 -->
  ...
</horizontal>
```

### Step 4: Launch 파일

**`simple_robot_spawn.launch.py`**:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_simple_robot = FindPackageShare('simple_robot_description')
    
    # URDF 처리
    robot_description = Command([
        'xacro', ' ',
        PathJoinSubstitution([
            pkg_simple_robot,
            'urdf',
            'simple_robot_with_uwb.urdf.xacro'
        ]),
        ' gazebo:=gz'
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }]
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-x', '5.0',
            '-y', '5.0',
            '-z', '0.05',
            '-topic', 'robot_description',
        ],
        output='screen',
    )
    
    # UWB Static TF
    uwb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'uwb_tag_link',
            'simple_robot/uwb_tag_link/uwb_tag'
        ]
    )
    
    # UWB Bridge
    uwb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uwb_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/model/simple_robot/uwb_tag_link/uwb_tag/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/model/simple_robot/uwb_tag_link/uwb_tag/scan', '/scan')
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
        uwb_tf,
        uwb_bridge,
    ])
```

### Step 5: AMCL 설정

**`simple_robot_localization.yaml`**:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    laser_model_type: "Tag&Anchors"
    
    # 앵커 좌표 (4개)
    anchors: [
       0.0,  0.0, 3.0,
      10.0,  0.0, 3.0,
      10.0, 10.0, 3.0,
       0.0, 10.0, 3.0
    ]
    
    # 노이즈 (URDF와 일치)
    sigma_hit: 0.2
    sigma_azimuth: 0.087
    
    # 파티클 필터
    max_particles: 2000
    min_particles: 500
    max_beams: 60
    
    # 모션 모델
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    
    # 업데이트
    update_min_d: 0.25
    update_min_a: 0.2
    resample_interval: 1
    
    # 측정 모델
    z_hit: 1.0
    z_max: 0.0
    z_rand: 0.0
    z_short: 0.0
    
    scan_topic: scan
```

### Step 6: 실행

```bash
# 1. Gazebo 시작
ros2 launch ros_gz_sim gz_sim.launch.py

# 2. 로봇 스폰
ros2 launch simple_robot_description simple_robot_spawn.launch.py

# 3. AMCL 실행
ros2 launch nav2_bringup localization_launch.py \
  params_file:=simple_robot_localization.yaml
```

---

## 트러블슈팅

### 문제 1: /scan 토픽이 발행되지 않음

**원인**:
- ros_gz_bridge 설정 오류
- 센서 이름 불일치

**해결**:

1. Gazebo 토픽 확인:
```bash
gz topic -l | grep scan
```

2. Bridge 토픽 확인:
```bash
ros2 topic list | grep scan
```

3. TF 트리 확인:
```bash
ros2 run tf2_tools view_frames
```

### 문제 2: AMCL이 발산함

**원인**:
- 앵커 좌표 불일치
- 노이즈 파라미터 불일치
- 초기 위치가 부정확

**해결**:

1. **앵커 좌표 확인**:
   - URDF `<anchor>` 태그
   - AMCL `anchors` 파라미터
   - 순서와 좌표가 정확히 일치하는지 확인

2. **노이즈 파라미터 확인**:
   - URDF `<noise>` 의 `<stddev>`
   - AMCL `sigma_hit`, `sigma_azimuth`

3. **초기 위치 설정**:
```bash
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
  }
}"
```

### 문제 3: TF 에러

**증상**: `tf2::LookupException` 또는 `tf2::ExtrapolationException`

**해결**:

1. Static TF 발행 확인:
```bash
ros2 run tf2_ros tf2_echo base_link uwb_tag_link
```

2. TF 트리 시각화:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

### 문제 4: 센서 데이터가 이상함

**증상**: 거리 값이 모두 `inf` 또는 `nan`

**원인**:
- 앵커 좌표가 잘못됨
- 로봇이 측정 범위 밖

**해결**:

1. 센서 시각화 활성화:
```xml
<visualize>1</visualize>
```

2. 센서 데이터 모니터링:
```bash
ros2 topic echo /scan --field ranges
```

---

## 체크리스트

### URDF 통합 체크리스트

- [ ] `uwb_multianchor/urdf/multianchor_sensor.urdf.xacro` 포함
- [ ] 센서를 적절한 링크에 부착
- [ ] 센서 위치(`origin`) 설정
- [ ] 앵커 좌표 설정 (환경에 맞게)
- [ ] 앵커 개수 설정 (`<samples>`)
- [ ] 노이즈 파라미터 설정

### Launch 파일 체크리스트

- [ ] Robot State Publisher (URDF 발행)
- [ ] Spawn Robot (Gazebo)
- [ ] Static TF Publisher (센서 링크)
- [ ] ros_gz_bridge (센서 토픽)
- [ ] 토픽 리매핑 (`/scan`)

### AMCL 체크리스트

- [ ] `laser_model_type: "Tag&Anchors"`
- [ ] `anchors` 배열 (URDF와 일치)
- [ ] `sigma_hit` (URDF와 일치)
- [ ] `sigma_azimuth` (URDF와 일치)
- [ ] `max_particles` 충분한 값
- [ ] `scan_topic: scan`

---

## 다음 단계

1. **테스트**: 시뮬레이션에서 로봇 이동 및 위치 추정 확인
2. **튜닝**: AMCL 파라미터 조정으로 성능 향상
3. **실제 하드웨어**: 실제 UWB 하드웨어와 통합
4. **센서 융합**: LiDAR 등 다른 센서와 결합

---

## 참고 자료

- **예제 코드**: `examples/turtlebot4/`
- **센서 URDF**: `urdf/multianchor_sensor.urdf.xacro`
- **사용 가이드**: `docs/USAGE.md`
- **README**: `README.md`

---

**질문이나 문제가 있으면 GitHub Issues에 등록해주세요!**

