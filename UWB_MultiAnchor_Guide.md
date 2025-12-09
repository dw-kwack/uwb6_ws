# UWB MultiAnchor 시스템 구현 가이드

## 프로젝트 개요

이 프로젝트는 TurtleBot4 시뮬레이션 환경에 UWB(Ultra-Wideband) 기반 MultiAnchor 측위 시스템을 통합한 것입니다. 기존의 LiDAR 기반 위치 추정 대신, 고정된 앵커(Anchor)들과 로봇에 탑재된 태그(Tag) 간의 거리 및 방위각 정보를 활용하여 로봇의 위치를 추정합니다.

## "by Noh"가 부착된 파일 목록

### 1. Gazebo Harmonic - Sensor 구현

#### **MultiAnchor 센서 핵심 파일**
- **`src/gazebo_harmonic/gz-sensors/include/gz/sensors/MultiAnchorSensor.hh`** (전체 파일)
  - MultiAnchor 센서의 헤더 파일
  - Lidar 클래스를 상속받아 구현
  
- **`src/gazebo_harmonic/gz-sensors/src/MultiAnchorSensor.cc`** (수정: 2줄)
  - MultiAnchor 센서의 구현 파일
  - 거리 및 방위각 계산 로직 포함

- **`src/gazebo_harmonic/gz-sim/include/gz/sim/components/MultiAnchor.hh`** (전체 파일)
  - Gazebo 시뮬레이터용 MultiAnchor 컴포넌트 정의

#### **Lidar 센서 확장**
- **`src/gazebo_harmonic/gz-sensors/include/gz/sensors/Lidar.hh`** (수정: 2곳)
  - Lidar 인터페이스 확장 (방위각 노이즈 지원)
  
- **`src/gazebo_harmonic/gz-sensors/src/Lidar.cc`** (수정: 9곳)
  - 방위각 정보 및 노이즈 처리 추가

- **`src/gazebo_harmonic/gz-sensors/include/gz/sensors/SensorTypes.hh`** (수정: 1곳)
  - MultiAnchor 센서 타입 등록

- **`src/gazebo_harmonic/gz-sensors/include/gz/sensors/GpuLidarSensor.hh`** (수정: 2곳)
  - 오타 수정 (_heighti → _height)

#### **SDF 파서 확장**
- **`src/gazebo_harmonic/sdformat/include/sdf/Lidar.hh`** (수정: 1곳)
  - 앵커 좌표 및 방위각 노이즈 파라미터 지원
  
- **`src/gazebo_harmonic/sdformat/src/Lidar.cc`** (수정: 4곳)
  - SDF에서 앵커 좌표와 방위각 노이즈 파싱

- **`src/gazebo_harmonic/sdformat/include/sdf/Sensor.hh`** (수정: 1곳)
  - MultiAnchor 센서 타입 추가
  
- **`src/gazebo_harmonic/sdformat/src/Sensor.cc`** (수정: 4곳)
  - MultiAnchor 센서 파싱 로직

### 2. Gazebo Harmonic - 시뮬레이터 통합

- **`src/gazebo_harmonic/gz-sim/src/rendering/RenderUtil.cc`** (수정: 6곳)
  - MultiAnchor 센서 렌더링 지원
  
- **`src/gazebo_harmonic/gz-sim/src/systems/sensors/Sensors.cc`** (수정: 5곳)
  - MultiAnchor 센서 시스템 통합
  
- **`src/gazebo_harmonic/gz-sim/src/Conversions.cc`** (수정: 2곳)
  - MultiAnchor 관련 데이터 변환
  
- **`src/gazebo_harmonic/gz-sim/src/SdfEntityCreator.cc`** (수정: 2곳)
  - MultiAnchor 엔티티 생성

#### **빌드 시스템**
- **`src/gazebo_harmonic/gz-sim/CMakeLists.txt`** (수정: 1곳)
- **`src/gazebo_harmonic/gz-sim/src/systems/sensors/CMakeLists.txt`** (수정: 1곳)
- **`src/gazebo_harmonic/gz-sensors/src/CMakeLists.txt`** (수정: 1곳)

### 3. Navigation2 - AMCL 확장

#### **Tag&Anchor 모델 구현**
- **`src/navigation2/nav2_amcl/src/sensors/laser/tag_anchor_model.cpp`** (전체 파일)
  - UWB Tag와 Anchor 간 거리/방위각 기반 파티클 필터 구현
  - 가우시안 노이즈 모델 적용
  
- **`src/navigation2/nav2_amcl/include/nav2_amcl/sensors/laser/laser.hpp`** (수정: 2곳)
  - TagAnchorModel 클래스 정의
  
- **`src/navigation2/nav2_amcl/src/amcl_node.cpp`** (수정: 8곳)
  - Tag&Anchor 모델 통합
  - 앵커 좌표 파라미터 처리
  
- **`src/navigation2/nav2_amcl/include/nav2_amcl/amcl_node.hpp`** (수정: 3곳)
  - 앵커 관련 파라미터 추가

#### **빌드 시스템**
- **`src/navigation2/nav2_amcl/src/sensors/CMakeLists.txt`** (수정: 2곳)

### 4. TurtleBot4 - 로봇 모델 및 설정

#### **URDF 파일**
- **`src/turtlebot4/turtlebot4/turtlebot4_description/urdf/sensors/multianchor.urdf.xacro`** (수정: 1곳)
  - MultiAnchor 센서 URDF 정의
  - 8개 앵커 좌표 설정
  - 거리/방위각 노이즈 파라미터
  
- **`src/turtlebot4/turtlebot4/turtlebot4_description/urdf/standard/turtlebot4_multianchor.urdf.xacro`** (수정: 3곳)
  - TurtleBot4 Standard 모델에 MultiAnchor 센서 추가
  
- **`src/turtlebot4/turtlebot4/turtlebot4_description/urdf/lite/turtlebot4_multianchor.urdf.xacro`** (수정: 3곳)
  - TurtleBot4 Lite 모델에 MultiAnchor 센서 추가

#### **Launch 파일**
- **`src/turtlebot4/turtlebot4_simulator/turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py`** (수정: 1곳)
  - MultiAnchor 센서용 static transform publisher 추가
  
- **`src/turtlebot4/turtlebot4_simulator/turtlebot4_gz_bringup/launch/ros_gz_bridge.launch.py`** (수정: 2곳)
  - MultiAnchor 토픽 브리징

#### **Navigation 설정**
- **`src/turtlebot4/turtlebot4/turtlebot4_navigation/config/localization_multianchor.yaml`** (수정: 3곳)
  - AMCL 파라미터 설정
  - laser_model_type: "Tag&Anchors"
  - 8개 앵커 좌표 설정
  - sigma_hit, sigma_azimuth 노이즈 파라미터

---

## 주요 기능 설명

### 1. MultiAnchor 센서 시스템

**목적**: UWB 기반 실내 측위를 위한 새로운 센서 타입 구현

**핵심 기능**:
- 고정된 앵커 위치로부터 태그까지의 3D 거리 측정
- 태그에서 앵커로의 방위각(azimuth) 측정
- 거리 및 방위각에 대한 가우시안 노이즈 적용
- LaserScan 메시지 형식으로 데이터 발행

**설정 예시** (`multianchor.urdf.xacro`):
```xml
<sensor name="multianchorsensor" type="multianchor">
  <update_rate>10.0</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>8</samples>  <!-- 앵커 개수 -->
      </horizontal>
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
      <stddev>0.2</stddev>  <!-- 거리 측정 오차 -->
    </noise>
    <noise>
      <target>azimuth</target>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.087</stddev>  <!-- 방위각 측정 오차 (5도) -->
    </noise>
    <anchors>
      <anchor xyz="-5.0 -11.25 5.0" />
      <anchor xyz="-5.0  -3.75 5.0" />
      <!-- ... 총 8개 앵커 ... -->
    </anchors>
  </lidar>
</sensor>
```

### 2. Tag&Anchor 위치 추정 모델

**목적**: Navigation2의 AMCL에 UWB 기반 위치 추정 기능 추가

**알고리즘**:
1. 각 파티클에 대해 앵커까지의 예상 거리/방위각 계산
2. 측정값과 예상값의 차이에 가우시안 확률 적용
3. 모든 앵커에 대한 확률을 곱하여 파티클 가중치 업데이트

**수식**:
- 거리 확률: `P(range) = (1/(σ_range * √(2π))) * exp(-0.5 * (err_range/σ_range)²)`
- 방위각 확률: `P(azimuth) = (1/(σ_azimuth * √(2π))) * exp(-0.5 * (err_azimuth/σ_azimuth)²)`
- 최종 가중치: `weight *= P(range) * P(azimuth)` (모든 앵커)

**설정 예시** (`localization_multianchor.yaml`):
```yaml
amcl:
  ros__parameters:
    laser_model_type: "Tag&Anchors"
    anchors: [-5.0, -11.25, 5.0,
              -5.0,  -3.75, 5.0,
              # ... 8개 앵커 좌표 ...
              5.0,  11.25, 5.0]
    sigma_hit: 0.2        # 거리 측정 표준편차
    sigma_azimuth: 0.087  # 방위각 측정 표준편차 (5도)
    max_beams: 60
    max_particles: 2000
```

### 3. Gazebo 시뮬레이터 통합

**구현 내용**:
- SDF 파서에 `<multianchor>` 센서 타입 추가
- `<anchors>` 태그로 앵커 좌표 정의
- `<noise target="azimuth">` 태그로 방위각 노이즈 정의
- Gazebo 렌더링 파이프라인에 MultiAnchor 센서 통합
- ROS 2 토픽으로 데이터 발행 (`/scan` 토픽)

---

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Harmonic                          │
│                                                             │
│  ┌──────────────────────────────────────────────┐          │
│  │  MultiAnchorSensor (gz-sensors)              │          │
│  │  - 앵커 좌표 관리                             │          │
│  │  - 거리/방위각 계산                           │          │
│  │  - 노이즈 적용                                │          │
│  └──────────────────┬───────────────────────────┘          │
│                     │ LaserScan 메시지                      │
└─────────────────────┼───────────────────────────────────────┘
                      │
                      │ ros_gz_bridge
                      │
┌─────────────────────▼───────────────────────────────────────┐
│                   ROS 2 Network                             │
│                   /scan topic                               │
└─────────────────────┬───────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────┐
│              Navigation2 - AMCL                             │
│                                                             │
│  ┌──────────────────────────────────────────────┐          │
│  │  TagAnchorModel                              │          │
│  │  - 파티클 필터                                │          │
│  │  - 가우시안 확률 모델                         │          │
│  │  - 위치 추정 (map → base_link)               │          │
│  └──────────────────┬───────────────────────────┘          │
│                     │ TF transform                          │
└─────────────────────┼───────────────────────────────────────┘
                      │
                      ▼
              로봇 위치 추정 완료
```

---

## 추가 기능 구현 가이드

### 방법 1: 앵커 개수 변경

**단계**:
1. **URDF 수정** (`multianchor.urdf.xacro`):
   ```xml
   <horizontal>
     <samples>12</samples>  <!-- 8 → 12로 변경 -->
   </horizontal>
   <anchors>
     <anchor xyz="x1 y1 z1" />
     <!-- ... 12개 앵커 정의 ... -->
   </anchors>
   ```

2. **AMCL 설정 수정** (`localization_multianchor.yaml`):
   ```yaml
   anchors: [x1, y1, z1,
             x2, y2, z2,
             # ... 12개 앵커 좌표 ...
             x12, y12, z12]
   ```

3. **빌드 및 실행**:
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch turtlebot4_gz_bringup sim.launch.py
   ```

### 방법 2: 노이즈 모델 변경

**현재**: 가우시안 노이즈만 지원

**추가 가능한 노이즈 모델**:
- **바이어스(Bias)**: 체계적 오차 추가
- **외부 간섭**: 특정 조건에서 측정 실패 모델링
- **다중경로(Multipath)**: 벽면 반사에 의한 오차

**구현 위치**:
- `src/gazebo_harmonic/gz-sensors/src/MultiAnchorSensor.cc`의 `SimulateMultiAnchorFrame()` 함수
- `src/navigation2/nav2_amcl/src/sensors/laser/tag_anchor_model.cpp`의 `sensorFunction()` 함수

**예시 코드**:
```cpp
// MultiAnchorSensor.cc - 바이어스 추가
double range_with_bias = true_range + gaussian_noise + range_bias;
double azimuth_with_bias = true_azimuth + gaussian_noise + azimuth_bias;
```

### 방법 3: 3D 위치 추정 확장

**현재**: 2D 평면 (x, y, θ)만 추정

**3D 확장**:
1. **높이(z) 정보 활용**:
   - 앵커 높이와 측정 거리로부터 로봇 높이 추정
   - AMCL 대신 3D 파티클 필터 사용

2. **구현 위치**:
   - `nav2_amcl/src/sensors/laser/tag_anchor_model.cpp`
   - 파티클 상태를 `(x, y, z, roll, pitch, yaw)`로 확장

### 방법 4: 실시간 앵커 추가/제거

**목적**: 시뮬레이션 중 동적으로 앵커 추가/제거

**구현 단계**:
1. **ROS 2 서비스 추가**:
   ```cpp
   // 서비스 정의
   service AddAnchor {
     geometry_msgs/Point position
     ---
     bool success
   }
   ```

2. **MultiAnchorSensor에 서비스 핸들러 추가**:
   ```cpp
   void MultiAnchorSensor::AddAnchorCallback(
       const AddAnchor::Request& req,
       AddAnchor::Response& res) {
     anchors_.push_back(req.position);
     res.success = true;
   }
   ```

3. **AMCL 노드에도 동일한 서비스 추가**

### 방법 5: 센서 융합

**목적**: LiDAR와 MultiAnchor를 함께 사용

**구현 방법**:
1. **AMCL에 하이브리드 모델 추가**:
   ```cpp
   class HybridModel : public Laser {
     LikelihoodFieldModel* lidar_model_;
     TagAnchorModel* uwb_model_;
     
     double weight_lidar = 0.5;
     double weight_uwb = 0.5;
     
     double sensorFunction() {
       double w_lidar = lidar_model_->sensorFunction();
       double w_uwb = uwb_model_->sensorFunction();
       return weight_lidar * w_lidar + weight_uwb * w_uwb;
     }
   };
   ```

2. **설정 파일에 가중치 파라미터 추가**:
   ```yaml
   laser_model_type: "Hybrid"
   lidar_weight: 0.5
   uwb_weight: 0.5
   ```

### 방법 6: 앵커 배치 최적화

**목적**: GDOP(Geometric Dilution of Precision) 최소화

**구현 도구**:
- Python 스크립트로 앵커 배치 시뮬레이션
- 최적화 알고리즘 (예: Simulated Annealing)

**평가 함수**:
```python
def compute_gdop(anchor_positions, robot_trajectory):
    total_gdop = 0
    for pose in robot_trajectory:
        H = compute_measurement_jacobian(anchor_positions, pose)
        G = np.linalg.inv(H.T @ H)
        gdop = np.sqrt(np.trace(G))
        total_gdop += gdop
    return total_gdop / len(robot_trajectory)
```

---

## 빌드 및 실행

### 빌드
```bash
cd /home/jiu-bae/uwb6_ws
colcon build --packages-up-to \
  gz-sensors8 \
  gz-sim8 \
  sdformat14 \
  nav2_amcl \
  turtlebot4_description \
  turtlebot4_gz_bringup

source install/setup.bash
```

### 실행

1. **시뮬레이터 실행**:
```bash
ros2 launch turtlebot4_gz_bringup sim.launch.py model:=standard
```

2. **MultiAnchor 기반 로컬라이제이션**:
```bash
ros2 launch turtlebot4_navigation localization.launch.py \
  params_file:=src/turtlebot4/turtlebot4/turtlebot4_navigation/config/localization_multianchor.yaml
```

3. **센서 데이터 확인**:
```bash
ros2 topic echo /scan
```

---

## 주의사항

### 1. 앵커 좌표 일치
- **URDF 파일**과 **AMCL 설정 파일**의 앵커 좌표가 반드시 일치해야 합니다.
- 순서도 동일해야 합니다.

### 2. 노이즈 파라미터 일치
- URDF의 `<noise>` 태그와 AMCL의 `sigma_hit`, `sigma_azimuth` 파라미터가 일치해야 합니다.
- 불일치 시 위치 추정 정확도가 크게 떨어집니다.

### 3. TF 프레임
- `multianchorsensor_link` TF가 올바르게 발행되는지 확인:
  ```bash
  ros2 run tf2_tools view_frames
  ```

### 4. 파티클 수
- 앵커 개수가 적을수록 더 많은 파티클이 필요합니다.
- 최소 권장: `max_particles: 2000`

---

## 트러블슈팅

### 문제 1: 위치 추정이 발산함
**원인**: 노이즈 파라미터 불일치 또는 앵커 좌표 오류

**해결**:
1. URDF와 AMCL 설정 파일 비교
2. `sigma_hit`, `sigma_azimuth` 값 확인
3. 앵커 좌표 순서 확인

### 문제 2: /scan 토픽이 발행되지 않음
**원인**: ros_gz_bridge 설정 누락

**해결**:
- `ros_gz_bridge.launch.py`에 MultiAnchor 토픽 브리징 추가:
```python
Bridge(
    '/model/turtlebot4/multianchorsensor_link/multianchorsensor/scan',
    'sensor_msgs/msg/LaserScan',
    LaserScan)
```

### 문제 3: Gazebo가 실행되지 않음
**원인**: MultiAnchor 센서 플러그인 빌드 실패

**해결**:
```bash
cd /home/jiu-bae/uwb6_ws
colcon build --packages-select gz-sensors8 --cmake-args -DCMAKE_BUILD_TYPE=Debug
cat log/latest_build/gz-sensors8/stdout_stderr.log
```

---

## 참고 자료

### 관련 파일 위치
- **센서 구현**: `src/gazebo_harmonic/gz-sensors/`
- **시뮬레이터 통합**: `src/gazebo_harmonic/gz-sim/`
- **AMCL 확장**: `src/navigation2/nav2_amcl/`
- **로봇 모델**: `src/turtlebot4/turtlebot4/turtlebot4_description/`
- **런치 파일**: `src/turtlebot4/turtlebot4_simulator/turtlebot4_gz_bringup/launch/`

### 주요 토픽
- `/scan`: MultiAnchor 센서 데이터 (sensor_msgs/LaserScan)
  - `ranges[i]`: i번째 앵커까지의 3D 거리
  - `intensities[i]`: i번째 앵커로의 방위각 (라디안)

### 주요 파라미터
- `update_rate`: 센서 업데이트 주기 (Hz)
- `sigma_hit`: 거리 측정 표준편차 (m)
- `sigma_azimuth`: 방위각 측정 표준편차 (rad)
- `max_particles`: AMCL 최대 파티클 수
- `anchors`: 앵커 좌표 배열 [x1,y1,z1, x2,y2,z2, ...]

---

## 결론

이 프로젝트는 UWB 기반 실내 측위 시스템을 ROS 2 및 Gazebo Harmonic 환경에 완전히 통합한 것입니다. LiDAR가 사용 불가능한 환경(예: 넓은 홀, 장애물이 적은 공간)에서 대안 또는 보완 수단으로 활용할 수 있습니다.

**핵심 장점**:
- 360도 전방향 측위 가능
- 장애물에 영향을 덜 받음
- LiDAR 대비 낮은 계산 비용

**향후 개선 방향**:
- 3D 위치 추정으로 확장
- LiDAR와 센서 융합
- 동적 앵커 관리
- NLOS(Non-Line-of-Sight) 오차 보정

---

**작성일**: 2025년 11월 27일  
**작성자**: AI Assistant  
**프로젝트 경로**: `/home/jiu-bae/uwb6_ws`

