# 🚀 TurtleBot4 + MultiAnchor 빠른 시작

## ⚡ 가장 빠른 실행 방법

### 옵션 1: 스크립트 사용 (권장 ⭐)

```bash
cd ~/uwb6_ws
./run_turtlebot4_multianchor.sh
```

### 옵션 2: 수동 실행 (2단계)

```bash
# 터미널 1: Gazebo
cd ~/uwb6_ws
source install/setup.bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 launch turtlebot4_gz_bringup sim.launch.py world:=warehouse

# 터미널 2: 로봇 스폰
cd ~/uwb6_ws
source install/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py model:=standard
```

---

## 🔍 확인 명령어

```bash
# MultiAnchor 센서 데이터 확인
ros2 topic echo /scan

# 토픽 목록
ros2 topic list | grep scan

# TF 확인
ros2 run tf2_tools view_frames
```

---

## 🐛 에러 발생 시

### "Failed to find a free participant index"

```bash
# DDS 설정 적용
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# 또는 2단계 실행 방법 사용
./run_turtlebot4_multianchor.sh --method 1
```

### Gazebo GUI 세그폴트

```bash
# Headless 모드로 실행
./run_turtlebot4_multianchor.sh --headless
```

### 기타 문제

```bash
# 모든 프로세스 종료
killall -9 ruby gz parameter_bridge

# 재실행
./run_turtlebot4_multianchor.sh
```

---

## 📚 상세 문서

- **[전체 실행 가이드](TURTLEBOT4_MULTIANCHOR_실행_가이드.md)** - 문제 해결 및 상세 설명
- **[uwb_multianchor 통합 가이드](src/uwb_multianchor/docs/INTEGRATION_GUIDE.md)** - 다른 로봇에 적용
- **[uwb_multianchor README](src/uwb_multianchor/README.md)** - 패키지 개요

---

## 💡 유용한 명령어

```bash
# 스크립트 옵션 보기
./run_turtlebot4_multianchor.sh --help

# Lite 모델 사용
./run_turtlebot4_multianchor.sh --model lite

# 다른 월드 사용
./run_turtlebot4_multianchor.sh --world depot

# 예제 Launch 사용
./run_turtlebot4_multianchor.sh --method 3
```

---

## ✅ 정상 작동 확인

실행 후 다음을 확인하세요:

1. ✅ Gazebo가 실행되고 로봇이 보임
2. ✅ `/scan` 토픽에 8개의 거리 값 (ranges)
3. ✅ `/scan` 토픽에 8개의 방위각 값 (intensities)
4. ✅ `multianchorsensor_link` TF 프레임 존재

---

**문제가 계속되면**: [TURTLEBOT4_MULTIANCHOR_실행_가이드.md](TURTLEBOT4_MULTIANCHOR_실행_가이드.md)를 참조하세요.

