# TurtleBot4 + UWB MultiAnchor 예제

이 디렉토리는 TurtleBot4 로봇에 UWB MultiAnchor 센서를 통합한 **참고 구현**입니다.

## ⚠️ 중요

이것은 **예제**입니다. `uwb_multianchor` 패키지는 TurtleBot4에 의존하지 않으며, 어떤 로봇에도 사용할 수 있습니다.

## 📁 파일 구조

```
turtlebot4/
├── urdf/
│   ├── turtlebot4_standard_multianchor.urdf.xacro
│   └── turtlebot4_lite_multianchor.urdf.xacro
├── launch/
│   ├── turtlebot4_multianchor_spawn.launch.py
│   └── turtlebot4_multianchor_sim.launch.py
└── config/
    └── localization_multianchor.yaml
```

## 🚀 사용법

### 방법 1: 예제 파일 직접 실행

```bash
# 환경 설정
source ~/ros2_ws/install/setup.bash

# 시뮬레이션 실행
ros2 launch \
  $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py

# 로컬라이제이션 (별도 터미널)
ros2 launch turtlebot4_navigation localization.launch.py \
  params_file:=$(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/config/localization_multianchor.yaml
```

### 방법 2: 자신의 패키지에 복사

```bash
# TurtleBot4 description 패키지로 복사
cp urdf/turtlebot4_standard_multianchor.urdf.xacro \
   ~/ros2_ws/src/turtlebot4/turtlebot4_description/urdf/

# Launch 파일 복사
cp launch/* \
   ~/ros2_ws/src/turtlebot4/turtlebot4_simulator/turtlebot4_gz_bringup/launch/

# Config 복사
cp config/localization_multianchor.yaml \
   ~/ros2_ws/src/turtlebot4/turtlebot4_navigation/config/
```

## 📝 파일 설명

### URDF 파일

#### `turtlebot4_standard_multianchor.urdf.xacro`
- TurtleBot4 Standard 모델 + UWB 센서
- 센서 위치: shell_link 기준 (0.00394, 0.0, 0.0753) m

#### `turtlebot4_lite_multianchor.urdf.xacro`
- TurtleBot4 Lite 모델 + UWB 센서
- 센서 위치: base_link 기준 (0.00394, 0.0, 0.0753) m

**핵심 코드**:
```xml
<xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />

<xacro:uwb_multianchor_sensor name="multianchorsensor" parent_link="shell_link">
  <origin xyz="0.00393584 0.0 0.07529272" rpy="0 0 1.5708"/>
</xacro:uwb_multianchor_sensor>
```

### Launch 파일

#### `turtlebot4_multianchor_sim.launch.py`
완전한 시뮬레이션 환경 실행:
- Gazebo 시작
- TurtleBot4 스폰 (MultiAnchor 포함)
- TurtleBot4 노드
- ros_gz_bridge

**파라미터**:
- `model`: `standard` 또는 `lite`
- `world`: `depot`, `maze`, `warehouse`
- `x`, `y`, `z`, `yaw`: 로봇 초기 위치

#### `turtlebot4_multianchor_spawn.launch.py`
로봇 스폰만 실행:
- Robot state publisher
- Spawn entity
- Static TF publisher
- ros_gz_bridge (MultiAnchor)

### Config 파일

#### `localization_multianchor.yaml`
AMCL 설정:
- `laser_model_type: "Tag&Anchors"`
- 8개 앵커 좌표
- 노이즈 파라미터 (`sigma_hit: 0.2`, `sigma_azimuth: 0.087`)
- 파티클 필터 파라미터

**앵커 배치** (Depot 월드 기준):
```
(-5, 11.25, 5)  ●─────●  (5, 11.25, 5)
                │     │
                │  🤖 │
                │     │
(-5,-11.25, 5)  ●─────●  (5,-11.25, 5)
```

## ⚙️ 커스터마이징

### 앵커 위치 변경

1. **URDF 수정** (`urdf/multianchor_sensor.urdf.xacro` 복사):
```xml
<anchors>
  <anchor xyz="x1 y1 z1" />
  <!-- ... -->
</anchors>
```

2. **AMCL 설정 수정** (`config/localization_multianchor.yaml`):
```yaml
anchors: [x1, y1, z1, x2, y2, z2, ...]
```

### 노이즈 레벨 조정

**URDF**:
```xml
<noise>
  <target>range</target>
  <stddev>0.15</stddev>  <!-- 0.2 → 0.15: 더 정확 -->
</noise>
```

**AMCL**:
```yaml
sigma_hit: 0.15  # URDF와 일치
```

## 🧪 테스트

### 1. 센서 데이터 확인
```bash
ros2 topic echo /scan
```

**예상 출력**:
```
ranges: [5.2, 4.8, 6.1, 5.9, 7.2, 6.8, 8.1, 7.9]  # 8개 거리
intensities: [1.57, 0.78, -0.52, -1.32, ...]       # 8개 방위각
```

### 2. TF 확인
```bash
ros2 run tf2_tools view_frames
# base_link → multianchorsensor_link 확인
```

### 3. AMCL 파티클 시각화
```bash
rviz2
# Add → ParticleCloud → Topic: /particle_cloud
```

## 🎓 학습 포인트

이 예제에서 배울 수 있는 것들:

1. **URDF 통합**: 기존 로봇에 센서 추가
2. **Launch 파일**: 복잡한 시스템 구성
3. **ros_gz_bridge**: Gazebo ↔ ROS 2 연결
4. **AMCL 설정**: Tag&Anchor 모델 사용
5. **TF 관리**: Static transform publisher

## 📚 다음 단계

### 다른 로봇에 적용하기

1. **[통합 가이드](../../docs/INTEGRATION_GUIDE.md) 읽기**
2. **이 예제 참조하여 자신의 로봇 URDF 수정**
3. **Launch 파일 작성 (이 예제 참조)**
4. **AMCL 설정 (이 예제 참조)**

### 성능 향상

- **앵커 배치 최적화**: GDOP 최소화
- **파라미터 튜닝**: `max_particles`, `update_min_d` 등
- **센서 융합**: LiDAR와 결합

## ⚠️ 주의사항

1. **의존성**: 이 예제를 사용하려면 TurtleBot4 패키지가 설치되어 있어야 합니다:
```bash
sudo apt install ros-$ROS_DISTRO-turtlebot4-desktop
```

2. **앵커 좌표 일치**: URDF와 AMCL 설정의 앵커 좌표가 정확히 일치해야 합니다.

3. **Gazebo 버전**: Gazebo Harmonic with MultiAnchor 센서 지원이 필요합니다.

## 🐛 문제 해결

### 문제: 로봇이 스폰되지 않음

**원인**: TurtleBot4 패키지 미설치

**해결**:
```bash
sudo apt install ros-$ROS_DISTRO-turtlebot4-simulator
```

### 문제: AMCL이 작동하지 않음

**원인**: Tag&Anchor 모델이 없는 nav2_amcl

**해결**: 커스텀 빌드된 nav2_amcl 필요 (워크스페이스의 것 사용)

## 📞 도움이 필요하신가요?

- 📖 [통합 가이드](../../docs/INTEGRATION_GUIDE.md)
- 📘 [메인 README](../../README.md)
- 🐛 [GitHub Issues](https://github.com/...)

---

**이 예제는 참고용입니다. 자신의 로봇에 맞게 수정하여 사용하세요!**

