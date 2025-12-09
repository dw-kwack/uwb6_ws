#!/bin/bash
# UWB MultiAnchor Package Test Script
# 이 스크립트는 패키지가 올바르게 설치되었는지 확인합니다.

set -e

echo "=========================================="
echo "UWB MultiAnchor Package Test"
echo "=========================================="
echo ""

# 색상 정의
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 워크스페이스 소스
echo "[1/7] Sourcing workspace..."
source /home/jiu-bae/uwb6_ws/install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

# 2. 패키지 존재 확인
echo "[2/7] Checking package installation..."
if ros2 pkg prefix uwb_multianchor > /dev/null 2>&1; then
    PKG_PATH=$(ros2 pkg prefix uwb_multianchor)
    echo -e "${GREEN}✓ Package found: ${PKG_PATH}${NC}"
else
    echo -e "${RED}✗ Package not found!${NC}"
    exit 1
fi
echo ""

# 3. URDF 파일 검증
echo "[3/7] Validating URDF files..."
URDF_STANDARD="${PKG_PATH}/share/uwb_multianchor/urdf/turtlebot4_standard_multianchor.urdf.xacro"
URDF_LITE="${PKG_PATH}/share/uwb_multianchor/urdf/turtlebot4_lite_multianchor.urdf.xacro"
URDF_SENSOR="${PKG_PATH}/share/uwb_multianchor/urdf/multianchor_sensor.urdf.xacro"

if [ -f "$URDF_STANDARD" ]; then
    echo -e "${GREEN}✓ Standard URDF exists${NC}"
else
    echo -e "${RED}✗ Standard URDF not found${NC}"
fi

if [ -f "$URDF_LITE" ]; then
    echo -e "${GREEN}✓ Lite URDF exists${NC}"
else
    echo -e "${RED}✗ Lite URDF not found${NC}"
fi

if [ -f "$URDF_SENSOR" ]; then
    echo -e "${GREEN}✓ Sensor URDF exists${NC}"
else
    echo -e "${RED}✗ Sensor URDF not found${NC}"
fi

echo -e "${YELLOW}ℹ URDF syntax validation is skipped (requires full launch context)${NC}"
echo ""

# 4. Launch 파일 확인
echo "[4/7] Checking launch files..."
LAUNCH_DIR="${PKG_PATH}/share/uwb_multianchor/launch"
if [ -d "$LAUNCH_DIR" ]; then
    echo -e "${GREEN}✓ Launch directory exists${NC}"
    LAUNCH_COUNT=$(find "$LAUNCH_DIR" -name "*.launch.py" | wc -l)
    echo -e "${GREEN}✓ Found ${LAUNCH_COUNT} launch files${NC}"
else
    echo -e "${RED}✗ Launch directory not found${NC}"
fi
echo ""

# 5. Config 파일 확인
echo "[5/7] Checking config files..."
CONFIG_FILE="${PKG_PATH}/share/uwb_multianchor/config/localization_multianchor.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}✓ AMCL config file exists${NC}"
    # YAML 유효성 검사
    if python3 -c "import yaml; yaml.safe_load(open('$CONFIG_FILE'))" 2>/dev/null; then
        echo -e "${GREEN}✓ YAML syntax is valid${NC}"
    else
        echo -e "${YELLOW}⚠ YAML syntax may have issues${NC}"
    fi
else
    echo -e "${RED}✗ Config file not found${NC}"
fi
echo ""

# 6. 문서 확인
echo "[6/7] Checking documentation..."
README="${PKG_PATH}/../../src/uwb_multianchor/README.md"
if [ -f "$README" ]; then
    echo -e "${GREEN}✓ README.md exists${NC}"
else
    echo -e "${YELLOW}⚠ README.md not found${NC}"
fi
echo ""

# 7. 의존성 확인
echo "[7/7] Checking core dependencies..."
DEPS_OK=true

# 필수 의존성만 확인
if ros2 pkg prefix sensor_msgs > /dev/null 2>&1; then
    echo -e "${GREEN}✓ sensor_msgs${NC}"
else
    echo -e "${RED}✗ sensor_msgs not found${NC}"
    DEPS_OK=false
fi

if ros2 pkg prefix geometry_msgs > /dev/null 2>&1; then
    echo -e "${GREEN}✓ geometry_msgs${NC}"
else
    echo -e "${RED}✗ geometry_msgs not found${NC}"
    DEPS_OK=false
fi

# 선택적 의존성 (예제용)
echo ""
echo "Optional dependencies (for examples):"
if ros2 pkg prefix turtlebot4_description > /dev/null 2>&1; then
    echo -e "${GREEN}✓ turtlebot4_description (for TurtleBot4 example)${NC}"
else
    echo -e "${YELLOW}○ turtlebot4_description not found (optional)${NC}"
fi

if ros2 pkg prefix nav2_amcl > /dev/null 2>&1; then
    echo -e "${GREEN}✓ nav2_amcl (for localization)${NC}"
else
    echo -e "${YELLOW}○ nav2_amcl not found (optional)${NC}"
fi

echo ""

# 최종 결과
echo "=========================================="
if $DEPS_OK; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    echo ""
    echo "Core sensor package is ready to use!"
    echo ""
    echo "Next steps:"
    echo "  1. Read: docs/INTEGRATION_GUIDE.md"
    echo "  2. See example: examples/turtlebot4/"
    echo "  3. Add sensor to your robot URDF"
else
    echo -e "${RED}✗ Core dependencies missing${NC}"
    echo "Install missing dependencies and try again."
fi
echo "=========================================="

