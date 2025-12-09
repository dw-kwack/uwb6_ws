#!/bin/bash

# TurtleBot4 + MultiAnchor 실행 스크립트
# 사용법: ./run_turtlebot4_multianchor.sh [옵션]

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 함수 정의
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

# 도움말
show_help() {
    cat << EOF
TurtleBot4 + MultiAnchor 실행 스크립트

사용법:
  $0 [옵션]

옵션:
  --method METHOD    실행 방법 선택 [1|2|3] (기본: 1)
                     1: 2단계 실행 (안정적, 권장)
                     2: 통합 실행 (DDS 설정 필요)
                     3: uwb_multianchor 예제 사용
  
  --model MODEL      로봇 모델 선택 [standard|lite] (기본: standard)
  --world WORLD      월드 선택 [depot|maze|warehouse] (기본: warehouse)
  --headless         GUI 없이 실행 (Headless 모드)
  --help             이 도움말 표시

예제:
  # 방법 1: 2단계 실행 (권장)
  $0 --method 1
  
  # 방법 2: 통합 실행
  $0 --method 2 --model standard
  
  # 방법 3: 예제 사용 (Lite 모델)
  $0 --method 3 --model lite
  
  # Headless 모드
  $0 --method 1 --headless

EOF
}

# 기본값 설정
METHOD="1"
MODEL="standard"
WORLD="warehouse"
HEADLESS="false"

# 인자 파싱
while [[ $# -gt 0 ]]; do
    case $1 in
        --method)
            METHOD="$2"
            shift 2
            ;;
        --model)
            MODEL="$2"
            shift 2
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        --headless)
            HEADLESS="true"
            shift
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            print_error "알 수 없는 옵션: $1"
            show_help
            exit 1
            ;;
    esac
done

# 워크스페이스 경로
WS_DIR="$HOME/uwb6_ws"

# 워크스페이스 확인
if [ ! -d "$WS_DIR" ]; then
    print_error "워크스페이스를 찾을 수 없습니다: $WS_DIR"
    exit 1
fi

# setup.bash 확인
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    print_error "워크스페이스가 빌드되지 않았습니다. 먼저 빌드하세요:"
    echo "  cd $WS_DIR && colcon build"
    exit 1
fi

# DDS 설정 확인
if [ ! -f "$HOME/cyclonedds.xml" ]; then
    print_info "DDS 설정 파일이 없습니다. 생성 중..."
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
    print_success "DDS 설정 파일 생성 완료: ~/cyclonedds.xml"
fi

# 환경 변수 설정
cd "$WS_DIR"
source install/setup.bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

if [ "$HEADLESS" = "true" ]; then
    export GZ_SIM_GUI=0
    export LIBGL_ALWAYS_SOFTWARE=1
fi

print_header "TurtleBot4 + MultiAnchor 시뮬레이션"
echo ""
echo "  방법: $METHOD"
echo "  모델: $MODEL"
echo "  월드: $WORLD"
echo "  Headless: $HEADLESS"
echo ""

# 실행 방법에 따라 분기
case $METHOD in
    1)
        print_header "방법 1: 2단계 실행 (권장)"
        print_info "먼저 Gazebo를 실행합니다..."
        echo ""
        print_info "다음 명령으로 로봇을 스폰하세요 (새 터미널):"
        echo ""
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py model:=$MODEL"
        echo ""
        print_info "Gazebo 실행 중..."
        
        ros2 launch turtlebot4_gz_bringup sim.launch.py world:=$WORLD
        ;;
        
    2)
        print_header "방법 2: 통합 실행"
        print_info "DDS 설정이 적용되었습니다."
        print_info "모든 노드를 한 번에 실행합니다..."
        
        ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
            model:=$MODEL \
            world:=$WORLD
        ;;
        
    3)
        print_header "방법 3: uwb_multianchor 예제 사용"
        
        # uwb_multianchor 패키지 확인
        if ! ros2 pkg prefix uwb_multianchor &> /dev/null; then
            print_error "uwb_multianchor 패키지를 찾을 수 없습니다."
            print_info "먼저 패키지를 빌드하세요:"
            echo "  cd $WS_DIR"
            echo "  colcon build --packages-select uwb_multianchor"
            exit 1
        fi
        
        print_info "uwb_multianchor 예제를 실행합니다..."
        
        LAUNCH_FILE="$(ros2 pkg prefix uwb_multianchor)/share/uwb_multianchor/examples/turtlebot4/launch/turtlebot4_multianchor_sim.launch.py"
        
        if [ ! -f "$LAUNCH_FILE" ]; then
            print_error "Launch 파일을 찾을 수 없습니다: $LAUNCH_FILE"
            exit 1
        fi
        
        ros2 launch "$LAUNCH_FILE" model:=$MODEL world:=$WORLD
        ;;
        
    *)
        print_error "잘못된 방법: $METHOD"
        print_info "사용 가능한 방법: 1, 2, 3"
        show_help
        exit 1
        ;;
esac

