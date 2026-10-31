#!/usr/bin/env bash
# 一键启动 robotarm
#
#   ./start.sh              ros2_control 模式（真实达妙 CAN 硬件）+ RViz   ← 默认
#   ./start.sh --no-rviz    不启动 RViz
#   ./start.sh --rebuild    先 colcon build
#   ./start.sh --rate 60    更新频率
#
# 启动前会自动配置 CAN 并清掉旧实例。

set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACE=can0
BITRATE=1000000
REBUILD=0
RVIZ=true
RATE=200

while [ $# -gt 0 ]; do
  case "$1" in
    --rebuild) REBUILD=1; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --rate)    RATE="$2"; shift 2 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

echo "═══ 1/4  CAN 接口 ═══"
if [ ! -e "/sys/class/net/$IFACE" ]; then
  echo "✗ 没有 $IFACE：PCAN 没插好或驱动没加载（sudo modprobe peak_usb）"
  exit 1
fi
if [ "$(cat /sys/class/net/$IFACE/operstate 2>/dev/null)" != "up" ]; then
  echo "  当前 DOWN，尝试自动拉起…"
  if sudo -n ip link set "$IFACE" down 2>/dev/null \
     && sudo -n ip link set "$IFACE" type can bitrate "$BITRATE" 2>/dev/null \
     && sudo -n ip link set "$IFACE" up 2>/dev/null; then
    echo "  ✓ 已配置 $BITRATE bps"
  else
    echo "✗ 需要 sudo，请手动执行三行后重跑："
    echo "    sudo ip link set $IFACE down"
    echo "    sudo ip link set $IFACE type can bitrate $BITRATE"
    echo "    sudo ip link set $IFACE up"
    exit 1
  fi
else
  echo "  ✓ $IFACE 已是 UP"
  if ip -details link show "$IFACE" 2>/dev/null | grep -q LOOPBACK; then
    echo "✗ $IFACE 处于 LOOPBACK 模式（帧不会上总线）"
    echo "    sudo ip link set $IFACE down"
    echo "    sudo ip link set $IFACE type can bitrate $BITRATE"
    echo "    sudo ip link set $IFACE up"
    exit 1
  fi
fi

echo "═══ 2/4  清理旧实例 ═══"
P_CTRL='^/opt/ros/jazzy/lib/controller_manager/ros2_control_node'
P_RSP='^/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher'
P_RVIZ='^/opt/ros/jazzy/lib/rviz2/rviz2'
P_LAUNCH='^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch robotarm ros2_control'

n=0
for p in $(pgrep -f "$P_LAUNCH" 2>/dev/null); do kill -INT "$p" 2>/dev/null && n=$((n+1)); done
[ "$n" -gt 0 ] && sleep 3
for pat in "$P_LAUNCH" "$P_CTRL" "$P_RSP" "$P_RVIZ"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do kill "$p" 2>/dev/null; done
done
sleep 1
for pat in "$P_LAUNCH" "$P_CTRL" "$P_RSP" "$P_RVIZ"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done
echo "  收到 $n 个旧 launch，其余残留已强制清理"

echo "═══ 3/4  环境 ═══"
cd "$WS"
[ ! -f install/setup.bash ] && { echo "  首次运行，先编译"; REBUILD=1; }
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ "$REBUILD" = "1" ]; then
  echo "  colcon build --symlink-install"
  colcon build --symlink-install 2>&1 | tail -2
fi
# shellcheck disable=SC1091
source install/setup.bash
echo "  ✓ 已 source"

echo "═══ 4/4  启动 ═══"
echo "  模式: ros2_control（硬件插件 robotarm_hardware/DmSystemHardware）  rviz=${RVIZ}"
exec ros2 launch robotarm ros2_control.launch.py rviz:="$RVIZ"
