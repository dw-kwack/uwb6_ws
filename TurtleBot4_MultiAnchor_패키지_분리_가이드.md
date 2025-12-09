# TurtleBot4 MultiAnchor 패키지 분리 가이드

## 1. 패키지 분리 가능 여부: ✅ **가능하며 권장됨**

### 분리의 장점
- ✅ **유지보수 용이**: 원본 TurtleBot4 패키지를 수정하지 않고 독립적으로 관리
- ✅ **업그레이드 안전**: TurtleBot4 패키지 업데이트 시 충돌 방지
- ✅ **재사용성**: 다른 로봇 플랫폼에도 적용 가능
- ✅ **배포 용이**: 독립적인 패키지로 GitHub/ROS Index에 공유 가능
- ✅ **의존성 명확화**: MultiAnchor 기능의 의존성을 명확히 관리

## 2. 현재 TurtleBot4 패키지 수정 사항

### 수정된 파일 목록
```
turtlebot4/
├── turtlebot4_description/
│   └── urdf/
│       ├── sensors/
│       │   └── multianchor.urdf.xacro (새로 추가)
│       ├── standard/
│       │   └── turtlebot4_multianchor.urdf.xacro (새로 추가)
│       └── lite/
│           └── turtlebot4_multianchor.urdf.xacro (새로 추가)
│
├── turtlebot4_navigation/
│   └── config/
│       └── localization_multianchor.yaml (새로 추가)
│
└── turtlebot4_simulator/
    └── turtlebot4_gz_bringup/
        └── launch/
            ├── turtlebot4_spawn.launch.py (수정: 1곳)
            └── ros_gz_bridge.launch.py (수정: 2곳)
```

## 3. 제안하는 새 패키지 구조

### 옵션 A: 단일 통합 패키지 (권장)

**패키지명**: `turtlebot4_multianchor`

```
turtlebot4_multianchor/
├── package.xml
├── CMakeLists.txt
├── README.md
├── urdf/
│   ├── multianchor_sensor.urdf.xacro          # 센서 정의
│   ├── turtlebot4_standard_multianchor.urdf.xacro
│   └── turtlebot4_lite_multianchor.urdf.xacro
├── launch/
│   ├── turtlebot4_multianchor_spawn.launch.py # 확장된 spawn
│   └── multianchor_bridge.launch.py           # ros_gz_bridge 확장
├── config/
│   └── localization_multianchor.yaml          # AMCL 설정
└── worlds/
    └── multianchor_depot.sdf                  # 앵커 포함 월드 (선택사항)
```

**장점**:
- 단순한 구조
- 한 곳에서 모든 MultiAnchor 관련 파일 관리
- 설치 및 배포 간편

### 옵션 B: 기능별 분리 패키지

**패키지 구조**:

1. **`multianchor_description`**: 센서 URDF 정의
   ```
   multianchor_description/
   ├── package.xml
   ├── CMakeLists.txt
   ├── urdf/
   │   └── multianchor_sensor.urdf.xacro
   └── meshes/
       └── multianchor.dae (선택사항)
   ```

2. **`turtlebot4_multianchor_description`**: TurtleBot4용 통합
   ```
   turtlebot4_multianchor_description/
   ├── package.xml
   ├── CMakeLists.txt
   └── urdf/
       ├── turtlebot4_standard_multianchor.urdf.xacro
       └── turtlebot4_lite_multianchor.urdf.xacro
   ```

3. **`turtlebot4_multianchor_bringup`**: Launch 파일
   ```
   turtlebot4_multianchor_bringup/
   ├── package.xml
   ├── CMakeLists.txt
   └── launch/
       ├── spawn.launch.py
       └── bridge.launch.py
   ```

4. **`turtlebot4_multianchor_navigation`**: Navigation 설정
   ```
   turtlebot4_multianchor_navigation/
   ├── package.xml
   ├── CMakeLists.txt
   └── config/
       └── localization_multianchor.yaml
   ```

**장점**:
- 모듈화된 구조
- 필요한 부분만 선택적으로 사용 가능
- 대규모 프로젝트에 적합

**단점**:
- 관리할 패키지가 많아짐
- 의존성 관계가 복잡해질 수 있음

## 4. 권장 구조: 옵션 A (단일 통합 패키지)

### 4.1. 패키지 메타데이터

**`package.xml`**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtlebot4_multianchor</name>
  <version>1.0.0</version>
  <description>
    MultiAnchor UWB positioning system integration for TurtleBot4.
    Provides sensor URDF, launch files, and navigation configuration.
  </description>
  
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  
  <author email="noh@email.com">Noh</author>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- TurtleBot4 dependencies -->
  <depend>turtlebot4_description</depend>
  <depend>turtlebot4_gz_bringup</depend>
  <depend>turtlebot4_navigation</depend>
  
  <!-- ROS 2 dependencies -->
  <depend>ros_gz_bridge</depend>
  <depend>tf2_ros</depend>
  <depend>sensor_msgs</depend>
  
  <!-- Gazebo dependencies -->
  <depend>gz_sensors8</depend>
  <depend>gz_sim8</depend>
  
  <!-- Navigation dependencies -->
  <depend>nav2_amcl</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**`CMakeLists.txt`**:
```cmake
cmake_minimum_required(VERSION 3.8)
project(turtlebot4_multianchor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 4.2. URDF 파일 구조

**`urdf/multianchor_sensor.urdf.xacro`**:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

<xacro:macro name="multianchor_sensor" params="name parent_link *origin">
  
  <!-- 센서 링크 정의 -->
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent_link}"/>
    <child link="${name}_link"/>
    <xacro:insert_block name="origin"/>
  </joint>
  
  <link name="${name}_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
      <material name="multianchor_material">
        <color rgba="0.1 0.1 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Gazebo 센서 정의 -->
  <gazebo reference="${name}_link">
    <sensor name="${name}" type="multianchor">
      <update_rate>10.0</update_rate>
      <visualize>0</visualize>
      <always_on>true</always_on>
      <lidar>
        <scan>
          <horizontal>
            <samples>8</samples>
            <resolution>1.0</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>0.0</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.2</min>
          <max>65.0</max>
          <resolution>0.05</resolution>
        </range>
        <noise>
          <target>range</target>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.2</stddev>
        </noise>
        <noise>
          <target>azimuth</target>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.087</stddev>
        </noise>
        <anchors>
          <anchor xyz="-5.0 -11.25 5.0" />
          <anchor xyz="-5.0  -3.75 5.0" />
          <anchor xyz="-5.0   3.75 5.0" />
          <anchor xyz="-5.0  11.25 5.0" />
          <anchor xyz=" 5.0 -11.25 5.0" />
          <anchor xyz=" 5.0  -3.75 5.0" />
          <anchor xyz=" 5.0   3.75 5.0" />
          <anchor xyz=" 5.0  11.25 5.0" />
        </anchors>
      </lidar>
    </sensor>
  </gazebo>
  
</xacro:macro>

</robot>
```

**`urdf/turtlebot4_standard_multianchor.urdf.xacro`**:
```xml
<?xml version="1.0"?>
<robot name="turtlebot4" xmlns:xacro="http://ros.org/wiki/xacro">
  
  <!-- 기존 TurtleBot4 Standard 포함 -->
  <xacro:include filename="$(find turtlebot4_description)/urdf/standard/turtlebot4.urdf.xacro" />
  
  <!-- MultiAnchor 센서 포함 -->
  <xacro:include filename="$(find turtlebot4_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <!-- 센서 위치 설정 -->
  <xacro:property name="multianchor_x_offset" value="0.00393584"/>
  <xacro:property name="multianchor_y_offset" value="0.0"/>
  <xacro:property name="multianchor_z_offset" value="0.07529272"/>
  
  <!-- MultiAnchor 센서 추가 -->
  <xacro:multianchor_sensor name="multianchor" parent_link="base_link">
    <origin xyz="${multianchor_x_offset} ${multianchor_y_offset} ${multianchor_z_offset}" 
            rpy="0 0 ${pi/2}"/>
  </xacro:multianchor_sensor>
  
</robot>
```

### 4.3. Launch 파일

**`launch/turtlebot4_multianchor_spawn.launch.py`**:
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    pkg_turtlebot4_gz_bringup = FindPackageShare('turtlebot4_gz_bringup')
    pkg_turtlebot4_multianchor = FindPackageShare('turtlebot4_multianchor')
    
    # Arguments
    robot_name = LaunchConfiguration('robot_name')
    model = LaunchConfiguration('model')
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot4',
        description='Robot name')
    
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model')
    
    # 기존 TurtleBot4 spawn (MultiAnchor URDF 사용)
    turtlebot4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_spawn.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'model': model,
        }.items()
    )
    
    # MultiAnchor 센서용 static transform
    multianchor_tf = Node(
        name='multianchor_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0.0',
            'multianchor_link',
            [robot_name, '/multianchor_link/multianchor']
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    # MultiAnchor 브리지
    multianchor_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot4_multianchor,
                'launch',
                'multianchor_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_robot_name)
    ld.add_action(declare_model)
    
    # Add actions
    ld.add_action(turtlebot4_spawn)
    ld.add_action(multianchor_tf)
    ld.add_action(multianchor_bridge)
    
    return ld
```

**`launch/multianchor_bridge.launch.py`**:
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    robot_name = LaunchConfiguration('robot_name')
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot4',
        description='Robot name')
    
    # MultiAnchor scan 토픽 브리지
    multianchor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multianchor_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            ['/model/', robot_name, '/multianchor_link/multianchor/scan',
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        ],
        remappings=[
            (['/model/', robot_name, '/multianchor_link/multianchor/scan'], '/scan')
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_robot_name)
    ld.add_action(multianchor_bridge)
    
    return ld
```

### 4.4. Config 파일

**`config/localization_multianchor.yaml`**:
```yaml
# MultiAnchor 기반 AMCL 설정
amcl:
  ros__parameters:
    use_sim_time: true
    
    # 모션 모델 파라미터
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    
    # 프레임 설정
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # Tag&Anchor 모델 설정
    laser_model_type: "Tag&Anchors"
    
    # 앵커 좌표 (x, y, z 순서)
    anchors: [
      -5.0, -11.25, 5.0,
      -5.0,  -3.75, 5.0,
      -5.0,   3.75, 5.0,
      -5.0,  11.25, 5.0,
       5.0, -11.25, 5.0,
       5.0,  -3.75, 5.0,
       5.0,   3.75, 5.0,
       5.0,  11.25, 5.0
    ]
    
    # 노이즈 파라미터 (URDF와 일치해야 함)
    sigma_hit: 0.2        # 거리 측정 표준편차 (m)
    sigma_azimuth: 0.087  # 방위각 측정 표준편차 (rad, 5도)
    
    # 파티클 필터 파라미터
    max_particles: 2000
    min_particles: 500
    max_beams: 60
    
    # 업데이트 파라미터
    update_min_a: 0.2
    update_min_d: 0.25
    resample_interval: 1
    
    # 측정 모델 가중치
    z_hit: 1.0
    z_max: 0.0
    z_rand: 0.0
    z_short: 0.0
    
    # 기타
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    tf_broadcast: true
    transform_tolerance: 1.0
    scan_topic: scan

# Map Server 설정
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""

# Map Saver 설정
map_saver:
  ros__parameters:
    use_sim_time: true
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true
```

### 4.5. README.md

**`README.md`**:
```markdown
# TurtleBot4 MultiAnchor

UWB 기반 MultiAnchor 측위 시스템을 TurtleBot4에 통합한 패키지입니다.

## 설치

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> turtlebot4_multianchor
cd ~/ros2_ws
colcon build --packages-select turtlebot4_multianchor
source install/setup.bash
```

## 사용법

### 1. MultiAnchor가 포함된 TurtleBot4 시뮬레이션 실행

```bash
ros2 launch turtlebot4_multianchor turtlebot4_multianchor_spawn.launch.py \
  model:=standard
```

### 2. MultiAnchor 기반 로컬라이제이션

```bash
ros2 launch turtlebot4_navigation localization.launch.py \
  params_file:=$(ros2 pkg prefix turtlebot4_multianchor)/share/turtlebot4_multianchor/config/localization_multianchor.yaml
```

### 3. 센서 데이터 확인

```bash
ros2 topic echo /scan
```

## 설정

### 앵커 위치 변경

`config/localization_multianchor.yaml` 및 `urdf/multianchor_sensor.urdf.xacro`의 앵커 좌표를 수정하세요.

### 노이즈 파라미터 조정

- URDF: `<noise>` 태그의 `<stddev>` 값
- AMCL: `sigma_hit`, `sigma_azimuth` 파라미터

두 값이 일치해야 정확한 위치 추정이 가능합니다.

## 의존성

- TurtleBot4 패키지
- Gazebo Harmonic
- Navigation2 (AMCL with Tag&Anchor model)
- gz_sensors8 (with MultiAnchor support)

## 라이선스

Apache 2.0

## 작성자

Noh - 원저작자
```

## 5. 패키지 분리 작업 단계

### Step 1: 패키지 생성

```bash
cd /home/jiu-bae/uwb6_ws/src
mkdir -p turtlebot4_multianchor/{urdf,launch,config,worlds}
cd turtlebot4_multianchor
```

### Step 2: 파일 이동 및 수정

```bash
# URDF 파일 복사
cp ../turtlebot4/turtlebot4/turtlebot4_description/urdf/sensors/multianchor.urdf.xacro \
   urdf/multianchor_sensor.urdf.xacro

# Launch 파일은 새로 작성 (위 템플릿 사용)

# Config 파일 복사
cp ../turtlebot4/turtlebot4/turtlebot4_navigation/config/localization_multianchor.yaml \
   config/
```

### Step 3: 파일 수정

1. **URDF 파일 수정**: 
   - `multianchor_sensor.urdf.xacro`를 독립적인 매크로로 변경
   - TurtleBot4 특화 요소 제거

2. **Launch 파일 작성**:
   - 위 템플릿 사용
   - 기존 TurtleBot4 launch를 wrapping

3. **통합 URDF 작성**:
   - `turtlebot4_standard_multianchor.urdf.xacro`
   - `turtlebot4_lite_multianchor.urdf.xacro`

### Step 4: 빌드 및 테스트

```bash
cd /home/jiu-bae/uwb6_ws
colcon build --packages-select turtlebot4_multianchor
source install/setup.bash

# 테스트
ros2 launch turtlebot4_multianchor turtlebot4_multianchor_spawn.launch.py
```

### Step 5: 원본 파일 정리 (선택사항)

패키지가 정상 작동하면 원본 TurtleBot4 패키지의 수정 사항을 제거할 수 있습니다:

```bash
# Backup 생성
cd /home/jiu-bae/uwb6_ws/src/turtlebot4
git checkout turtlebot4/turtlebot4_description/urdf/standard/turtlebot4_multianchor.urdf.xacro
git checkout turtlebot4/turtlebot4_description/urdf/lite/turtlebot4_multianchor.urdf.xacro
# ... (수정된 파일들 복원)
```

## 6. 패키지 사용 방법 비교

### 기존 방식 (TurtleBot4 패키지 수정)

```bash
# 단점: TurtleBot4 패키지 업데이트 시 수정 사항 손실 위험
ros2 launch turtlebot4_gz_bringup sim.launch.py model:=standard
```

### 새로운 방식 (독립 패키지)

```bash
# 장점: 독립적으로 관리, 업그레이드 안전
ros2 launch turtlebot4_multianchor turtlebot4_multianchor_spawn.launch.py model:=standard
```

## 7. 추가 개선 사항

### 7.1. 다중 로봇 지원

여러 로봇이 각자 다른 MultiAnchor 센서를 가질 수 있도록 네임스페이스 지원:

```python
# launch 파일에 추가
namespace = LaunchConfiguration('namespace')

declare_namespace = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Robot namespace')
```

### 7.2. 동적 앵커 설정

Launch 파일에서 앵커 좌표를 파라미터로 받기:

```python
declare_anchors = DeclareLaunchArgument(
    'anchors',
    default_value='[-5.0,-11.25,5.0, -5.0,-3.75,5.0, ...]',
    description='Anchor positions as comma-separated list')
```

### 7.3. 월드 파일 통합

앵커 위치를 시각화하는 마커를 월드 파일에 추가:

```xml
<!-- worlds/multianchor_depot.sdf -->
<model name="anchor_0">
  <pose>-5.0 -11.25 5.0 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <sphere><radius>0.1</radius></sphere>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## 8. 배포 및 공유

### GitHub 저장소 구조

```
turtlebot4_multianchor/
├── .github/
│   └── workflows/
│       └── ci.yml              # CI/CD 파이프라인
├── urdf/
├── launch/
├── config/
├── worlds/
├── docs/
│   ├── installation.md
│   ├── usage.md
│   └── api.md
├── package.xml
├── CMakeLists.txt
├── README.md
├── LICENSE
└── CHANGELOG.md
```

### ROS Index 등록

1. GitHub에 릴리즈 생성
2. [ROS Index](https://index.ros.org)에 패키지 등록
3. `rosdep` 의존성 관리

## 9. 마이그레이션 체크리스트

- [ ] 새 패키지 디렉토리 생성
- [ ] `package.xml` 작성
- [ ] `CMakeLists.txt` 작성
- [ ] URDF 파일 이동 및 수정
- [ ] Launch 파일 작성
- [ ] Config 파일 복사
- [ ] README.md 작성
- [ ] 빌드 테스트
- [ ] 시뮬레이션 실행 테스트
- [ ] Navigation 테스트
- [ ] 문서 작성
- [ ] Git 저장소 초기화
- [ ] (선택) 원본 TurtleBot4 패키지 수정 사항 제거

## 10. 결론

**권장 사항**: ✅ **옵션 A (단일 통합 패키지) 구조로 분리**

**이유**:
1. 관리가 간단함
2. 의존성이 명확함
3. 배포 및 공유가 용이함
4. TurtleBot4 업그레이드 시 안전함
5. 다른 로봇 플랫폼으로 확장 가능

**예상 작업 시간**: 2-4 시간

**난이도**: 중급 (ROS 2 패키지 구조 이해 필요)

---

이 가이드를 따라 패키지를 분리하면 유지보수가 훨씬 편리해지고, 다른 사용자들과 공유하기도 쉬워집니다!

