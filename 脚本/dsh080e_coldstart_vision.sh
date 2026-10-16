#!/bin/bash
# DSH-080e：方案 A 完整执行 —— 全链冷启动 + rcs 注入副本（EKF2 带 AID_MASK=8 初始化）
# 依据：GPT-079 三节批准方案 A；此处因 mavlink_interface 插件需重建 TCP 4560，必须重生成模型
BASE=/home/min/yeslab_build
PX4="$BASE/PX4-Autopilot"
BUILD="$PX4/build/px4_sitl_default"
SHIM="$BASE/sim_shim"
LOG="$BASE/_downloads/logs"
MODELS="$BASE/sim_models"
WORLD="/home/min/yeslab_build/sim_models/vins_world/vins_features.world"
VCFG="$BASE/vins_ws/src/VINS-Fusion/config/gazebo_stereo_imu_config.yaml"
RCS="$BASE/rcS_vision_startup.sh"
OUT=/home/min/dsh080e_coldstart_vision.txt

export DISPLAY=:0
export XAUTHORITY=/home/min/.Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
export ROS_MASTER_URI=http://localhost:11311
export GAZEBO_MODEL_PATH="$MODELS:$PX4/Tools/sitl_gazebo/models"
export GAZEBO_PLUGIN_PATH="$BUILD/build_gazebo"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:$BUILD/build_gazebo:$LD_LIBRARY_PATH"
export PATH="$SHIM:$PATH"
export PX4_SIM_MODEL=iris
source /opt/ros/noetic/setup.bash 2>/dev/null

getp () { timeout 12 rosservice call /mavros/param/get "param_id: '$1'" 2>&1 | grep -A3 value | grep -E 'integer|real' | tr -d ' ' | tr '\n' ' '; }

{
echo "=== 方案 A 完整执行：冷启动 + rcS 注入副本 $(date '+%F %T') ==="

echo ""
echo "──── [0] 前置校验 ────"
echo "  注入副本存在: $([ -f "$RCS" ] && echo yes || echo NO)  ($(wc -l < "$RCS" 2>/dev/null) 行)"
echo "  注入内容:"; grep -n "EKF2_" "$RCS" 2>/dev/null | sed 's/^/    /'
echo "  原 rcS md5（应始终不变）: $(md5sum "$BUILD/etc/init.d-posix/rcS" | awk '{print $1}')"
[ -f "$RCS" ] || { echo "  ❌ 注入副本缺失，终止"; exit 1; }

echo ""
echo "──── [1] 停止整链（保留 rosmaster，遵守 VINS headless 纪律）────"
pkill -x px4 2>/dev/null; pkill -f "px4-simulator" 2>/dev/null
pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
pkill -x vins_node 2>/dev/null; pkill -f "vins_to_px4.py" 2>/dev/null
sleep 6
echo "  残留: gzserver=$(pgrep -x gzserver | wc -l) px4=$(pgrep -x px4 | wc -l) vins=$(pgrep -x vins_node | wc -l)"
echo "  rosmaster: $(pgrep -f 'rosmaster --core' | head -1)"

echo ""
echo "──── [2] 启动 gzserver ────"
cd "$BUILD/tmp" || exit 1
setsid nohup gzserver "$WORLD" > "$LOG/dsh080e_gzserver.log" 2>&1 < /dev/null &
for i in $(seq 1 30); do sleep 2; gz topic -l >/dev/null 2>&1 && { echo "  ✅ gazebo master 就绪（$((i*2))s）"; break; }; done
sleep 4
echo "  URI 解析错误: $(grep -c 'Unable to find uri' "$LOG/dsh080e_gzserver.log" 2>/dev/null)"

echo ""
echo "──── [3] 重新生成模型 iris_vins（重建 mavlink_interface → TCP 4560）────"
gz model --spawn-file="$MODELS/iris_vins/iris_vins.sdf" --model-name=iris_vins -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -iE "error|Msg" | head -6 | sed 's/^/    /'
sleep 8
echo "  gazebo 相机话题: $(gz topic -l 2>/dev/null | grep -ic camera)"
echo "  模型在场景: $(gz model -m iris_vins --info 2>/dev/null | grep -c 'name:')"

echo ""
echo "──── [4] ★ 以 rcS 注入副本启动 PX4（EKF2 将带 AID_MASK=8/ HGT_MODE=3 初始化）────"
setsid nohup bash -c "tail -f /dev/null | '$BUILD/bin/px4' '$BUILD/etc' -s '$RCS' -t '$PX4/test_data'" \
  > "$LOG/dsh080e_px4.log" 2>&1 < /dev/null &
sleep 35
echo "  px4 进程: $(pgrep -x px4 | tr '\n' ' ')"
echo "  --- 启动日志关键行 ---"
grep -iE "Calling startup script|autostart|param.*Loaded|simulator|Exiting|ERROR" "$LOG/dsh080e_px4.log" 2>/dev/null | head -12 | sed 's/^/    /'

echo ""
echo "──── [5] MAVROS 重连 ────"
for i in $(seq 1 12); do
  C=$(timeout 8 rostopic echo -n1 /mavros/state 2>/dev/null | grep -o "connected: [A-Za-z]*")
  [ "$C" = "connected: True" ] && { echo "  ✅ MAVROS 已连接（${i}0s）"; break; }
  sleep 5
done
timeout 8 rostopic echo -n1 /mavros/state 2>&1 | grep -E "connected|armed|mode" | sed 's/^/    /'

echo ""
echo "──── [6] ★ 参数真实读数（此时 connected=True，非缓存）────"
echo "  EKF2_AID_MASK = $(getp EKF2_AID_MASK)"
echo "  EKF2_HGT_MODE = $(getp EKF2_HGT_MODE)"
echo "  EKF2_EV_DELAY = $(getp EKF2_EV_DELAY)"

echo ""
echo "──── [7] 恢复 VINS + 桥接 ────"
setsid nohup bash -c "source /opt/ros/noetic/setup.bash; source '$BASE/vins_ws/devel/setup.bash'; rosrun vins vins_node '$VCFG'" \
  > "$LOG/dsh080e_vins.log" 2>&1 < /dev/null &
sleep 30
echo "  vins_node = $(pgrep -x vins_node | head -1)"
setsid nohup bash -c "source /opt/ros/noetic/setup.bash; python3 /home/min/vins_to_px4.py" \
  > "$LOG/dsh080e_bridge.log" 2>&1 < /dev/null &
sleep 14
echo "  桥接 = $(pgrep -f vins_to_px4.py | head -1)"

echo ""
echo "──── [8] ★★ 关键验证：local_position 是否出现（EKF 是否接受 EV 水平位置）────"
for i in $(seq 1 8); do
  L=$(timeout 8 rostopic echo -n1 /mavros/local_position/pose 2>/dev/null | grep -A4 'position' | grep -E 'x:|y:|z:' | tr -d ' xyz' | tr '\n' ' ')
  echo "    ${i}：local_position(x,y,z)=(${L:-<无>})"
  sleep 6
done
echo "  local_position 频率: $(timeout 12 rostopic hz -w 5 /mavros/local_position/pose 2>&1 | grep -m1 'average rate' || echo '无输出')"

echo ""
echo "──── [9] 各通道复核 ────"
for t in /clock /imu0 /stereo/left/image_raw /vins_estimator/odometry /mavros/vision_pose/pose /mavros/global_position/global; do
  echo "  $t : $(timeout 10 rostopic hz -w 5 $t 2>&1 | grep -m1 'average rate' | tr -s ' ')"
done

echo ""
echo "──── [10] 重启后参数存档 ────"
timeout 20 rosservice call /mavros/param/pull "{}" 2>&1 | tr -s ' ' | tr '\n' ' ' | sed 's/^/  pull: /'
echo ""
rosparam dump /home/min/DSH079_after_restart.yaml /mavros 2>&1 | sed 's/^/  /'
grep -E "EKF2_AID_MASK|EKF2_HGT_MODE|EKF2_EV_DELAY" /home/min/DSH079_after_restart.yaml 2>/dev/null | sed 's/^/    /'

echo ""
echo "──── [11] 进程总览 ────"
ps -eo pid,etimes,comm --no-headers | grep -E 'rosmaster|gzserver|px4|vins_node|mavros|python3' | sed 's/^/  /'
echo "  负载: $(uptime)"

echo ""
echo "=== 结束 $(date '+%F %T') ==="
} > "$OUT" 2>&1
sync
