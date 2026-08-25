#!/usr/bin/env bash
set -euo pipefail

ROBOT_PROFILE="${1:-${PINKK_ROBOT_PROFILE:-robot_a}}"
case "${ROBOT_PROFILE}" in
    robot_a) PROFILE_DOMAIN_ID=36 ;;
    robot_b) PROFILE_DOMAIN_ID=38 ;;
    *)
        echo "지원하지 않는 로봇 프로필입니다: ${ROBOT_PROFILE} (robot_a|robot_b)" >&2
        exit 2
        ;;
esac

source_environment() {
    local setup_file="$1"
    if [[ ! -f "${setup_file}" ]]; then
        echo "환경 파일이 없습니다: ${setup_file}" >&2
        exit 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
}

source_environment /opt/ros/jazzy/setup.bash
ROBOT_VENV="${PINKK_ROBOT_VENV:-${HOME}/venv/mycobot}"
ROBOT_INSTALL_SETUP="${PINKK_ROBOT_INSTALL_SETUP:-${HOME}/mycobot_moveit_ws/install_pinkk/setup.bash}"
source_environment "${ROBOT_VENV}/bin/activate"
source_environment "${ROBOT_INSTALL_SETUP}"

# colcon이 만든 ROS console script는 /usr/bin/python3을 사용한다.
# PyMyCobot 가상환경의 패키지를 시스템 Python entry point에서도 찾게 한다.
PYMYCOBOT_PYTHON="${ROBOT_VENV}/bin/python"
if ! "${PYMYCOBOT_PYTHON}" -c "from pymycobot import MyCobot280" 2>/dev/null; then
    echo "PyMyCobot 가상환경에 pymycobot이 없습니다: ${PYMYCOBOT_PYTHON}" >&2
    exit 1
fi
PYMYCOBOT_SITE_PACKAGES="$(
    "${PYMYCOBOT_PYTHON}" -c \
        "import site; print(site.getsitepackages()[0])"
)"
export PYTHONPATH="${PYMYCOBOT_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

export ROS_DOMAIN_ID="${PINKK_ROS_DOMAIN_ID:-${PROFILE_DOMAIN_ID}}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "로봇 프로필=${ROBOT_PROFILE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 launch pinkk_usb_insertion observe_session.launch.py \
    robot_profile:="${ROBOT_PROFILE}"
