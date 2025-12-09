# TurtleBot4 + MultiAnchor 실행 가이드

## 🐛 문제 분석

로그를 분석한 결과 다음과 같은 문제가 발견되었습니다:

### 1. DDS Participant 한계 에러
```
Failed to find a free participant index for domain 0
```

**원인**: 너무 많은 ROS 2 노드가 동시에 실행되어 DDS participant 한계 초과 (45개 이상의 프로세스)

### 2. OpenGL 세그멘테이션 폴트
```
libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device.
Segmentation fault (Address not mapped to object [0x8])
```

**원인**: Gazebo GUI 렌더링 문제

### 3. 성공한 부분 ✅
- MultiAnchor 센서가 정상 인식됨
- 로봇과 도킹 스테이션이 성공적으로 스폰됨
- 컨트롤러 활성화 성공

---

## ✅ 올바른 실행 방법

### 방법 1: 2단계 실행 (권장)

DDS participant 한계를 피하기 위해 Gazebo를 먼저 실행하고, 로봇을 별도로 스폰합니다.

#### Step 1: Gazebo 실행

```bash
# 터미널 1
cd ~/uwb6_ws
source install/setup.bash

# Gazebo Harmonic 실행 (GUI만)
ros2 launch turtlebot4_gz_bringup sim.launch.py world:=warehouse
```

**참고**: GUI 세그폴트가 발생하면 무시하고 다시 실행하거나, headless 모드 사용:
```bash
GZ_SIM_RESOURCE_PATH=~/uwb6_ws/install/turtlebot4_gz_bringup/share/turtlebot4_gz_bringup/worlds gz sim warehouse.sdf -r -v 4 -s
```

#### Step 2: 로봇 스폰 (MultiAnchor 포함)

```bash
# 터미널 2
cd ~/uwb6_ws
source install/setup.bash

# Standard 모델 + MultiAnchor
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py model:=standard

# 또는 Lite 모델 + MultiAnchor
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py model:=lite
```

---

### 방법 2: 통합 실행 (DDS 설정 필요)

한 번에 모든 것을 실행하려면 DDS participant 한계를 늘려야 합니다.

#### Step 1: DDS 설정 수정

**Cyclone DDS (기본) 사용 시**:

```bash
# Cyclone DDS 설정 파일 생성
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

# 환경 변수 설정
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**Fast DDS 사용 시**:

```bash
# Fast DDS 설정 파일 생성
cat > ~/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <discoveryServersList>
                        <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>127.0.0.1</address>
                                        <port>11811</port>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </RemoteServer>
                    </discoveryServersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

# 환경 변수 설정
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml
```

#### Step 2: 통합 실행

```bash
cd ~/uwb6_ws
source install/setup.bash

# DDS 환경 변수 설정 (Cyclone 또는 Fast DDS)
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# 통합 실행
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=standard world:=warehouse
```

---

### 방법 3: uwb_multianchor 예제 사용 (가장 간단)

범용 패키지의 예제를 사용합니다.

```bash
cd ~/uwb6_ws
source install/setup.bash

# 전체 시스템 실행
ros2 launch \
  $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py \
  model:=standard \
  world:=warehouse
```

---

## 🔍 확인 방법

### 1. MultiAnchor 센서 토픽 확인

```bash
# 토픽 리스트 확인
ros2 topic list | grep scan

# 출력 예상:
# /scan  <- MultiAnchor 센서 데이터
# /lidar_scan  <- RPLiDAR 데이터
```

### 2. 센서 데이터 확인

```bash
# MultiAnchor 데이터 (8개 앵커)
ros2 topic echo /scan

# 예상 출력:
# ranges: [d1, d2, d3, d4, d5, d6, d7, d8]  # 8개 거리
# intensities: [θ1, θ2, θ3, θ4, θ5, θ6, θ7, θ8]  # 8개 방위각
```

### 3. Gazebo 센서 확인

```bash
# Gazebo 토픽 확인
gz topic -l | grep multianchor

# 출력 예상:
# /world/warehouse/model/turtlebot4/link/multianchorsensor_link/sensor/multianchorsensor/scan
```

### 4. TF 확인

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# 확인 항목:
# - base_link → multianchorsensor_link
```

---

## 🐛 트러블슈팅

### 문제 1: "Failed to find a free participant index"

**해결**:
1. **방법 1 사용** (2단계 실행)
2. DDS 설정 파일 생성 및 적용 (위 참조)
3. 불필요한 ROS 2 노드 종료:
```bash
# 모든 ROS 2 프로세스 종료
killall -9 ruby
killall -9 parameter_bridge
killall -9 gz
```

### 문제 2: Gazebo GUI 세그폴트

**해결**:
1. **소프트웨어 렌더링 강제**:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot4_gz_bringup sim.launch.py
```

2. **Headless 모드** (GUI 없이):
```bash
# 방법 1: 환경 변수
export GZ_SIM_GUI=0
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# 방법 2: gz sim 직접 실행
gz sim warehouse.sdf -r -s
```

3. **그래픽 드라이버 업데이트**:
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

### 문제 3: MultiAnchor 센서가 보이지 않음

**확인**:
```bash
# 1. 로봇 URDF 확인
ros2 topic echo /robot_description --once | grep multianchor

# 2. Gazebo 월드 확인
gz model -m turtlebot4 -i

# 3. 센서 목록 확인
gz topic -l | grep sensor
```

**해결**:
- 올바른 URDF가 사용되는지 확인
- `turtlebot4_standard_multianchor.urdf.xacro` 또는 `turtlebot4_lite_multianchor.urdf.xacro` 사용

### 문제 4: 로봇이 스폰되지 않음

**해결**:
```bash
# 1. Gazebo가 실행 중인지 확인
gz world list

# 2. 수동 스폰
gz model --spawn-file=<path-to-urdf> -m turtlebot4

# 3. 로그 확인
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py --show-args
```

---

## 📊 리소스 사용량

### 정상 실행 시 (방법 1):

| 구성 요소 | 프로세스 수 | 메모리 |
|---------|-----------|--------|
| Gazebo Sim | 1 | ~2GB |
| ros_gz_bridge | ~25 | ~500MB |
| TurtleBot4 노드 | ~15 | ~300MB |
| **합계** | **~41** | **~2.8GB** |

### DDS Participant 사용:
- **기본 한계**: 약 120개
- **실제 사용**: 41개 (정상)
- **에러 발생**: 120개 초과 시

---

## 🎯 권장 실행 순서

### 개발/테스트 시 (방법 1 권장):

```bash
# 1. Gazebo 실행
ros2 launch turtlebot4_gz_bringup sim.launch.py world:=warehouse

# 2. 로봇 스폰 (새 터미널)
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py model:=standard

# 3. Navigation 실행 (선택, 새 터미널)
ros2 launch turtlebot4_navigation localization.launch.py \
  params_file:=$(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/config/localization_multianchor.yaml
```

### 데모/프레젠테이션 시 (방법 3 권장):

```bash
# DDS 설정
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# 한 번에 실행
ros2 launch \
  $(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py
```

---

## 📝 정리

| 방법 | 장점 | 단점 | 권장 사용 |
|------|------|------|----------|
| **1. 2단계 실행** | 안정적, DDS 문제 없음 | 2번 실행 필요 | ✅ 개발/테스트 |
| **2. 통합 실행** | 한 번에 실행 | DDS 설정 필요 | 🔧 설정 후 사용 |
| **3. 예제 사용** | 가장 간단 | uwb_multianchor 필요 | 🎯 데모/학습 |

---

## 🔗 참고 링크

- [TurtleBot4 공식 문서](https://turtlebot.github.io/turtlebot4-user-manual/)
- [uwb_multianchor 통합 가이드](src/uwb_multianchor/docs/INTEGRATION_GUIDE.md)
- [Gazebo Harmonic 문서](https://gazebosim.org/docs/harmonic/install)
- [Cyclone DDS 설정](https://github.com/eclipse-cyclonedds/cyclonedds)

---

**작성일**: 2025년 11월 28일  
**워크스페이스**: `/home/jiu-bae/uwb6_ws`  
**ROS 버전**: ROS 2 Jazzy  
**Gazebo 버전**: Harmonic (8.9.0)

