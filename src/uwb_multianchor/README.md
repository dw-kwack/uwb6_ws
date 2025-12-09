# UWB MultiAnchor Sensor Package

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble+-blue)](https://docs.ros.org/en/humble/)

**범용 UWB 기반 MultiAnchor 측위 센서 패키지**  
모든 로봇 플랫폼에 적용 가능한 플러그인 방식의 UWB 센서 시스템

---

## 🎯 개요

`uwb_multianchor`는 **로봇 플랫폼에 독립적인** UWB(Ultra-Wideband) 기반 실내 측위 센서 패키지입니다. 간단한 URDF 매크로 호출만으로 어떤 로봇에도 MultiAnchor 센서를 추가할 수 있습니다.

### 핵심 특징

- 🤖 **범용 센서**: 어떤 로봇 플랫폼에도 적용 가능
- 🔌 **플러그인 방식**: URDF 파일에 한 줄만 추가
- 📡 **8개 앵커 지원**: 고정된 앵커로부터 3D 거리 및 방위각 측정
- 🎮 **Gazebo Harmonic 완벽 통합**: 실시간 시뮬레이션
- 🗺️ **Navigation2 호환**: AMCL Tag&Anchor 센서 모델
- 📚 **완전한 문서화**: 단계별 통합 가이드 제공

### 작동 원리

```
고정 앵커 (천장/벽면)
    ●──────●
    │      │
    │  🤖  │  ← 로봇 (UWB 태그)
    │      │
    ●──────●

거리 + 방위각 측정 → AMCL → 위치 추정
```

---

## 🚀 빠른 시작

### 3줄로 센서 추가하기

```xml
<!-- 1. 센서 매크로 포함 -->
<xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />

<!-- 2. 로봇에 센서 부착 -->
<xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
  <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
</xacro:uwb_multianchor_sensor>
```

**끝!** 이제 로봇이 UWB 측위를 사용할 수 있습니다.

---

## 📦 설치

### 방법 1: 소스에서 빌드 (권장)

```bash
cd ~/ros2_ws/src
git clone <repository-url> uwb_multianchor
cd ~/ros2_ws
colcon build --packages-select uwb_multianchor
source install/setup.bash
```

### 방법 2: 로컬 복사

```bash
cp -r /path/to/uwb_multianchor ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select uwb_multianchor
source install/setup.bash
```

### 의존성

**필수**:
- ROS 2 Humble 이상
- Gazebo Harmonic (MultiAnchor 센서 지원)
  - `gz-sensors8` with MultiAnchor
  - `gz-sim8` with MultiAnchor
  - `sdformat14` with MultiAnchor

**선택** (Navigation 사용 시):
- Navigation2 with Tag&Anchor model
  - `nav2_amcl` (커스텀 빌드 필요)

**로봇별 의존성**:
- 사용할 로봇의 description 패키지
  - 예: `turtlebot4_description`, `my_robot_description`

---

## 📖 사용법

### Step 1: 로봇 URDF에 센서 추가

**예제: `my_robot.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 기존 로봇 정의 -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.urdf.xacro" />
  
  <!-- UWB MultiAnchor 센서 추가 -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <xacro:uwb_multianchor_sensor name="uwb_tag" parent_link="base_link">
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </xacro:uwb_multianchor_sensor>

</robot>
```

### Step 2: 앵커 좌표 설정

센서 매크로 파일(`urdf/multianchor_sensor.urdf.xacro`)에서 앵커 위치를 환경에 맞게 수정:

```xml
<anchors>
  <anchor xyz="0.0  0.0  3.0" />  <!-- 앵커 1 (x, y, z in meters) -->
  <anchor xyz="5.0  0.0  3.0" />  <!-- 앵커 2 -->
  <anchor xyz="5.0  5.0  3.0" />  <!-- 앵커 3 -->
  <anchor xyz="0.0  5.0  3.0" />  <!-- 앵커 4 -->
  <!-- 최대 8개까지 추가 가능 -->
</anchors>
```

### Step 3: Launch 파일 작성

```python
from launch_ros.actions import Node

# Robot State Publisher
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': '...'}]
)

# UWB Bridge (Gazebo → ROS 2)
uwb_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/model/my_robot/uwb_tag_link/uwb_tag/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    ],
    remappings=[('/model/my_robot/uwb_tag_link/uwb_tag/scan', '/scan')]
)
```

### Step 4: AMCL 설정 (Navigation 사용 시)

```yaml
amcl:
  ros__parameters:
    laser_model_type: "Tag&Anchors"
    
    # 앵커 좌표 (URDF와 일치해야 함!)
    anchors: [
      0.0, 0.0, 3.0,
      5.0, 0.0, 3.0,
      5.0, 5.0, 3.0,
      0.0, 5.0, 3.0
    ]
    
    # 노이즈 파라미터 (URDF와 일치)
    sigma_hit: 0.2        # 거리 측정 표준편차 (m)
    sigma_azimuth: 0.087  # 방위각 측정 표준편차 (rad)
    
    max_particles: 2000
    # ... 기타 AMCL 파라미터
```

---

## 📚 문서

### 주요 문서

1. **[통합 가이드](docs/INTEGRATION_GUIDE.md)** ⭐ - **필독!**
   - 단계별 센서 추가 방법
   - AMCL 설정 가이드
   - TurtleBot4 예제
   - 커스텀 로봇 예제

2. **[사용 가이드](docs/USAGE.md)**
   - 고급 설정
   - 파라미터 튜닝
   - 트러블슈팅

### 빠른 링크

- 📁 **[TurtleBot4 예제](examples/turtlebot4/)** - 실제 구현 예제
- 🤖 **[커스텀 로봇 템플릿](examples/custom_robot/)** - 새 로봇 시작점
- 🔧 **[센서 URDF](urdf/multianchor_sensor.urdf.xacro)** - 센서 정의

---

## 🎓 예제

### 예제 1: TurtleBot4 (완전 구현)

TurtleBot4에 MultiAnchor 센서를 추가한 완전한 예제입니다.

**위치**: `examples/turtlebot4/`

**파일**:
- `urdf/turtlebot4_standard_multianchor.urdf.xacro`
- `urdf/turtlebot4_lite_multianchor.urdf.xacro`
- `launch/turtlebot4_multianchor_sim.launch.py`
- `config/localization_multianchor.yaml`

**실행**:
```bash
# 전체 시스템 실행 (예제 포함)
ros2 launch \
  $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py
```

### 예제 2: 간단한 로봇

**시나리오**: 0.5m x 0.4m 차동구동 로봇

```xml
<robot name="simple_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  
  <!-- 로봇 베이스 -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </visual>
  </link>
  
  <!-- UWB 센서 추가 (로봇 위 15cm) -->
  <xacro:include filename="$(find uwb_multianchor)/urdf/multianchor_sensor.urdf.xacro" />
  
  <xacro:uwb_multianchor_sensor name="uwb" parent_link="base_link">
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </xacro:uwb_multianchor_sensor>
  
</robot>
```

더 많은 예제는 [통합 가이드](docs/INTEGRATION_GUIDE.md)를 참조하세요.

---

## 🔧 설정

### 센서 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `update_rate` | 10.0 Hz | 센서 업데이트 주기 |
| `range.min` | 0.2 m | 최소 측정 거리 |
| `range.max` | 65.0 m | 최대 측정 거리 |
| `noise.range.stddev` | 0.2 m | 거리 측정 노이즈 |
| `noise.azimuth.stddev` | 0.087 rad | 방위각 노이즈 (5도) |

### 앵커 배치 권장사항

| 환경 크기 | 앵커 개수 | 배치 방법 |
|----------|----------|----------|
| < 5m x 5m | 4개 | 모서리 |
| 5m ~ 10m | 4-6개 | 모서리 + 중간 |
| > 10m | 6-8개 | 균등 분포 |

**팁**: GDOP(Geometric Dilution of Precision)를 최소화하도록 배치

---

## 📊 토픽

### Published

- `/scan` (`sensor_msgs/LaserScan`)
  - MultiAnchor 센서 데이터
  - `ranges[i]`: i번째 앵커까지의 거리 (m)
  - `intensities[i]`: i번째 앵커로의 방위각 (rad)

### TF Frames

```
map → odom → base_link → uwb_tag_link
```

---

## 🎯 지원 로봇

이 패키지는 **모든 로봇 플랫폼**과 호환됩니다:

### 테스트된 로봇
- ✅ TurtleBot4 (Standard, Lite)
- ✅ 커스텀 차동구동 로봇

### 호환 가능 (미테스트)
- TurtleBot3
- Create3
- ROSbot
- Husky
- Clearpath Jackal
- **당신의 로봇!**

**통합 방법**: [통합 가이드](docs/INTEGRATION_GUIDE.md) 참조

---

## 🛠️ 개발

### 패키지 구조

```
uwb_multianchor/
├── urdf/
│   └── multianchor_sensor.urdf.xacro    # 핵심 센서 매크로
├── examples/
│   ├── turtlebot4/                       # TurtleBot4 예제
│   │   ├── urdf/
│   │   ├── launch/
│   │   └── config/
│   └── custom_robot/                     # 템플릿
├── docs/
│   ├── INTEGRATION_GUIDE.md              # 통합 가이드
│   └── USAGE.md                          # 사용 가이드
├── package.xml                           # 의존성 (로봇 무관)
└── CMakeLists.txt
```

### 센서 매크로 커스터마이징

1. **로봇별 센서 파일 생성**:
```bash
cp urdf/multianchor_sensor.urdf.xacro \
   ~/my_robot_description/urdf/my_robot_uwb.urdf.xacro
```

2. **앵커 좌표 수정**
3. **노이즈 파라미터 조정**
4. **로봇 URDF에서 사용**

---

## 🐛 트러블슈팅

### 문제: /scan 토픽이 없음

**해결**:
1. ros_gz_bridge 실행 확인
2. 토픽 이름 확인: `gz topic -l | grep scan`
3. TF 프레임 확인: `ros2 run tf2_tools view_frames`

### 문제: AMCL 발산

**해결**:
1. 앵커 좌표 일치 확인 (URDF vs AMCL config)
2. 노이즈 파라미터 일치 확인
3. 파티클 수 증가 (`max_particles: 3000`)

더 많은 해결책: [통합 가이드 - 트러블슈팅](docs/INTEGRATION_GUIDE.md#트러블슈팅)

---

## 🤝 기여

기여는 언제나 환영합니다!

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

### 기여 아이디어

- [ ] 새로운 로봇 예제 추가
- [ ] Rviz 플러그인 (앵커 시각화)
- [ ] 3D 위치 추정 확장
- [ ] 실제 UWB 하드웨어 지원
- [ ] 다중 로봇 협동 측위

---

## 📄 라이선스

이 프로젝트는 Apache 2.0 라이선스 하에 배포됩니다.

```
Copyright 2025 Noh

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
```

---

## 👥 작성자

- **Noh** - 원저작자 및 개발자
- **jiu-bae** - 유지보수

---

## 📞 지원

### 문서
- 📖 [통합 가이드](docs/INTEGRATION_GUIDE.md) - 새 로봇에 센서 추가
- 📘 [사용 가이드](docs/USAGE.md) - 고급 사용법

### 커뮤니티
- GitHub Issues - 버그 리포트 및 기능 요청
- GitHub Discussions - 질문 및 토론

### 예제
- `examples/turtlebot4/` - TurtleBot4 완전 구현
- `examples/custom_robot/` - 새 로봇 템플릿

---

## 🌟 핵심 장점

| 기존 접근 방식 | uwb_multianchor |
|--------------|-----------------|
| ❌ 로봇별로 센서 구현 | ✅ 한 번 구현, 모든 로봇에 적용 |
| ❌ 복잡한 통합 과정 | ✅ 3줄로 센서 추가 |
| ❌ 로봇 패키지 수정 필요 | ✅ 원본 패키지 유지 |
| ❌ 업그레이드 시 충돌 | ✅ 독립적 업데이트 |

---

## 📈 로드맵

### v2.0 (현재) ✅
- [x] 로봇 독립적 패키지
- [x] 범용 센서 매크로
- [x] TurtleBot4 예제
- [x] 완전한 통합 가이드

### v2.1 (예정)
- [ ] Rviz 플러그인
- [ ] 더 많은 로봇 예제
- [ ] 단위 테스트
- [ ] CI/CD

### v3.0 (계획)
- [ ] 3D 위치 추정
- [ ] LiDAR 센서 융합
- [ ] 실제 하드웨어 지원
- [ ] 다중 로봇 협동

---

## 💡 FAQ

**Q: 어떤 로봇에 사용할 수 있나요?**  
A: URDF를 사용하는 모든 ROS 2 로봇에 사용 가능합니다.

**Q: TurtleBot4 패키지가 필요한가요?**  
A: 아니요! 예제로만 제공되며, 어떤 로봇 패키지 없이도 센서 매크로를 사용할 수 있습니다.

**Q: 실제 UWB 하드웨어와 호환되나요?**  
A: 현재는 시뮬레이션만 지원합니다. 실제 하드웨어 지원은 v3.0에서 계획 중입니다.

**Q: 앵커를 4개보다 적게 사용할 수 있나요?**  
A: 이론적으로는 3개면 충분하지만, 정확도를 위해 4개 이상을 권장합니다.

**Q: Navigation2 없이 사용할 수 있나요?**  
A: 네! 센서 데이터만 필요하다면 AMCL 없이도 사용 가능합니다.

---

<div align="center">

**🚀 지금 바로 당신의 로봇에 UWB 측위 추가하기!**

[통합 가이드 보기](docs/INTEGRATION_GUIDE.md) | [예제 보기](examples/) | [기여하기](#기여)

</div>
