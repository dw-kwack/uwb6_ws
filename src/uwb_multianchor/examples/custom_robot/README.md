# 커스텀 로봇 템플릿

이 디렉토리는 **새로운 로봇**에 UWB MultiAnchor 센서를 추가할 때 사용할 수 있는 템플릿과 가이드를 제공합니다.

## 🎯 목표

자신의 로봇에 3단계로 UWB 센서 추가:
1. URDF에 센서 추가
2. Launch 파일 작성
3. AMCL 설정 (Navigation 사용 시)

## 📋 체크리스트

시작하기 전에 준비할 것:

- [ ] 로봇 URDF 파일
- [ ] 로봇이 Gazebo에서 작동함
- [ ] 센서를 부착할 링크 이름 (`base_link`, `body_link` 등)
- [ ] 환경의 앵커 좌표 (3D 위치)
- [ ] (선택) 맵 파일 (Navigation 사용 시)

## 🚀 빠른 시작

### Step 1: URDF 템플릿

**`my_robot_with_uwb.urdf.xacro`**:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- ========================================
       기존 로봇 정의 (변경하지 마세요)
       ======================================== -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.urdf.xacro" />
  
  <!-- ========================================
       UWB 센서 추가 (아래만 수정하세요)
       ======================================== -->
  
  <!-- 1. UWB 센서 매크로 포함 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <!-- 2. 센서 위치 설정 -->
  <xacro:property name="uwb_x" value="0.0"/>    <!-- 전후 오프셋 -->
  <xacro:property name="uwb_y" value="0.0"/>    <!-- 좌우 오프셋 -->
  <xacro:property name="uwb_z" value="0.1"/>    <!-- 상하 오프셋 -->
  <xacro:property name="uwb_yaw" value="0.0"/>  <!-- 회전 -->
  
  <!-- 3. 센서 부착 -->
  <xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
    <origin xyz="${uwb_x} ${uwb_y} ${uwb_z}" 
            rpy="0 0 ${uwb_yaw}"/>
  </xacro:uwb_multianchor_sensor>

</robot>
```

**수정할 항목**:
1. `my_robot_description` → 자신의 패키지명
2. `my_robot.urdf.xacro` → 자신의 URDF 파일명
3. `parent_link="base_link"` → 센서를 부착할 링크
4. `uwb_x`, `uwb_y`, `uwb_z` → 센서 위치

### Step 2: Launch 템플릿

**`my_robot_spawn.launch.py`**:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # ========================================
    # 수정 필요: 패키지명
    # ========================================
    pkg_my_robot = FindPackageShare('my_robot_description')
    
    # ========================================
    # 수정 필요: URDF 파일 경로
    # ========================================
    robot_description = Command([
        'xacro', ' ',
        PathJoinSubstitution([
            pkg_my_robot,
            'urdf',
            'my_robot_with_uwb.urdf.xacro'  # <- 수정
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
    
    # ========================================
    # 수정 필요: 로봇 이름 및 초기 위치
    # ========================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',  # <- 로봇 이름
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
            '-topic', 'robot_description',
        ],
        output='screen',
    )
    
    # ========================================
    # 수정 필요: 센서 링크 이름
    # ========================================
    uwb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'uwb_tag_link',                    # <- URDF의 센서 링크명
            'my_robot/uwb_tag_link/uwb_tag'   # <- 로봇 이름 + 센서 경로
        ]
    )
    
    # ========================================
    # 수정 필요: 로봇 이름
    # ========================================
    uwb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uwb_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/model/my_robot/uwb_tag_link/uwb_tag/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
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

**수정할 항목**:
1. `my_robot_description` → 패키지명
2. `my_robot_with_uwb.urdf.xacro` → URDF 파일명
3. `my_robot` → 로봇 이름 (3곳)
4. `uwb_tag_link` → 센서 링크명

### Step 3: AMCL 설정 템플릿

**`my_robot_localization.yaml`**:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    
    # ========================================
    # 프레임 IDs (로봇에 맞게 수정)
    # ========================================
    base_frame_id: "base_link"       # <- 로봇의 베이스 링크
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # ========================================
    # UWB 센서 모델 (변경 금지!)
    # ========================================
    laser_model_type: "Tag&Anchors"
    
    # ========================================
    # 앵커 좌표 (환경에 맞게 수정 필수!)
    # ========================================
    # 형식: [x1, y1, z1, x2, y2, z2, ...]
    # 단위: 미터 (m)
    # 주의: URDF의 앵커 좌표와 정확히 일치해야 함!
    anchors: [
      0.0,  0.0, 3.0,   # 앵커 1
      5.0,  0.0, 3.0,   # 앵커 2
      5.0,  5.0, 3.0,   # 앵커 3
      0.0,  5.0, 3.0    # 앵커 4
    ]
    
    # ========================================
    # 노이즈 파라미터 (URDF와 일치해야 함!)
    # ========================================
    sigma_hit: 0.2        # 거리 측정 표준편차 (m)
    sigma_azimuth: 0.087  # 방위각 측정 표준편차 (rad, 5도)
    
    # ========================================
    # 파티클 필터 파라미터
    # ========================================
    max_particles: 2000
    min_particles: 500
    max_beams: 60
    
    # ========================================
    # 모션 모델 (로봇 타입에 맞게)
    # ========================================
    robot_model_type: "nav2_amcl::DifferentialMotionModel"  # 또는 OmniMotionModel
    
    # 모션 노이즈
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    
    # 업데이트 임계값
    update_min_d: 0.25    # 최소 이동 거리 (m)
    update_min_a: 0.2     # 최소 회전 각도 (rad)
    resample_interval: 1
    
    # 측정 모델 가중치
    z_hit: 1.0
    z_max: 0.0
    z_rand: 0.0
    z_short: 0.0
    
    # 기타
    pf_err: 0.05
    pf_z: 0.99
    save_pose_rate: 0.5
    tf_broadcast: true
    transform_tolerance: 1.0
    scan_topic: scan
```

**수정할 항목**:
1. `base_frame_id` → 로봇의 베이스 프레임
2. `anchors` → 환경의 앵커 좌표
3. `sigma_hit`, `sigma_azimuth` → URDF와 일치
4. `robot_model_type` → 로봇 모션 모델

## 📝 상세 가이드

### 앵커 좌표 설정

#### 1. 환경 측정

실제 환경에서 앵커 위치를 측정:

```
앵커 배치 예시 (5m x 5m 방):

      (0,5,3)           (5,5,3)
         ●─────────────────●
         │                 │
         │                 │
         │    (2.5,2.5,0)  │  <- 로봇 시작
         │       🤖        │
         │                 │
         ●─────────────────●
      (0,0,3)           (5,0,3)
```

#### 2. URDF에 입력

센서 파일 복사 및 수정:

```bash
cp $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/urdf/multianchor_sensor.urdf.xacro \
   ~/my_robot_description/urdf/my_robot_uwb_sensor.urdf.xacro
```

수정:
```xml
<anchors>
  <anchor xyz="0.0 0.0 3.0" />
  <anchor xyz="5.0 0.0 3.0" />
  <anchor xyz="5.0 5.0 3.0" />
  <anchor xyz="0.0 5.0 3.0" />
</anchors>

<horizontal>
  <samples>4</samples>  <!-- 앵커 개수 -->
</horizontal>
```

#### 3. AMCL에 입력

```yaml
anchors: [
  0.0, 0.0, 3.0,
  5.0, 0.0, 3.0,
  5.0, 5.0, 3.0,
  0.0, 5.0, 3.0
]
```

### 노이즈 파라미터 튜닝

#### 거리 노이즈 (`sigma_hit`)

| 값 (m) | 의미 | 적용 |
|--------|------|------|
| 0.1 | 매우 정확 | 고급 UWB |
| 0.2 | 일반적 | 대부분 |
| 0.5 | 부정확 | 저가 센서 |

#### 방위각 노이즈 (`sigma_azimuth`)

| 값 (rad) | 각도 | 적용 |
|---------|------|------|
| 0.0436 | 2.5° | 매우 정확 |
| 0.087 | 5° | 일반적 |
| 0.175 | 10° | 부정확 |

**중요**: URDF와 AMCL의 값이 일치해야 합니다!

## 🧪 테스트 방법

### 1. URDF 검증

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro my_robot_with_uwb.urdf.xacro gazebo:=gz)"

# 다른 터미널에서
ros2 run tf2_tools view_frames
# uwb_tag_link가 있는지 확인
```

### 2. Gazebo 테스트

```bash
# Gazebo 시작
ros2 launch ros_gz_sim gz_sim.launch.py

# 로봇 스폰
ros2 launch my_robot_description my_robot_spawn.launch.py

# 센서 데이터 확인
ros2 topic echo /scan
```

### 3. AMCL 테스트

```bash
# AMCL 실행
ros2 launch nav2_bringup localization_launch.py \
  params_file:=my_robot_localization.yaml

# 초기 위치 설정
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "..."

# Rviz에서 확인
rviz2
```

## 📚 더 알아보기

- **[통합 가이드](../../docs/INTEGRATION_GUIDE.md)** - 상세한 단계별 가이드
- **[TurtleBot4 예제](../turtlebot4/)** - 실제 구현 예제
- **[메인 README](../../README.md)** - 패키지 개요

## 💡 팁

1. **센서 위치**: 로봇의 중심 위쪽에 배치
2. **앵커 배치**: 로봇 작업 영역을 둘러싸도록
3. **테스트**: 작은 환경에서 먼저 테스트
4. **튜닝**: 초기엔 `max_particles`를 높게 설정

## 🐛 문제 해결

### 문제: /scan 토픽이 없음

```bash
# Gazebo 토픽 확인
gz topic -l | grep scan

# ROS 토픽 확인
ros2 topic list | grep scan

# Bridge 로그 확인
ros2 run ros_gz_bridge parameter_bridge --ros-args --log-level debug
```

### 문제: AMCL 발산

1. 앵커 좌표 재확인 (URDF vs AMCL)
2. `max_particles`를 3000으로 증가
3. 초기 위치를 수동으로 설정

## 📞 도움 요청

막히셨나요? 다음 리소스를 확인하세요:

- 📖 [통합 가이드](../../docs/INTEGRATION_GUIDE.md) - 자세한 설명
- 🐛 [트러블슈팅](../../docs/INTEGRATION_GUIDE.md#트러블슈팅) - 일반적인 문제
- 💬 GitHub Issues - 커뮤니티 도움

---

**행운을 빕니다! 🚀**

