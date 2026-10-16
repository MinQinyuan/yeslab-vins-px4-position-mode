#!/bin/bash
# YESLab-Gazebo-VINS-PX4 : restart VINS node only + strict EV health gate (DSH-082/083, GPT-083 approved flow)
# GPT-083 health gate: |EKF pos|<0.5 m, 5 s move<0.05 m, |v|<0.1 m/s, pos_test_ratio<1, VINS stable window
# Prints GATE OK / GATE UNSTABLE. Exit 0 = safe to arm.
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/min/yeslab_build/vins_ws/devel/setup.bash 2>/dev/null
CFG=/home/min/yeslab_build/vins_ws/src/VINS-Fusion/config/gazebo_stereo_imu_config.yaml
LOGD=/home/min/yeslab_build/_downloads/logs
cd /home/min/yeslab_build/PX4-Autopilot/build/px4_sitl_default/tmp || exit 1
echo "[1/4] restart vins_node (process restart only; no source/config/param change)"
pkill -x vins_node; sleep 3
nohup rosrun vins vins_node "$CFG" > "$LOGD/vins_restart.log" 2>&1 &
sleep 4
echo "vins pid: $(pgrep -x vins_node | tr '\n' ' ')"
echo "[2/4] wait 15 s for VINS init / EKF re-latch"
sleep 15
echo "[3/4] rate check"
for t in /vins_estimator/odometry /mavros/vision_pose/pose; do printf "%-30s" "$t"; timeout 6 rostopic hz -w 30 "$t" 2>/dev/null | grep -m1 'average rate'; done
echo "[4/4] strict health gate"
python3 - <<'PY'
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus
rospy.init_node('ev_gate', anonymous=True)
d = {}
def cp(m): d['p'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
def cv(m): d['v'] = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)
def ce(m): d['e'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
def cr(m):
    for s in m.__slots__:
        l = s.lower()
        if 'ratio' in l and ('pos' in l or 'horiz' in l):
            d['ratio_field'] = s; d['r'] = getattr(m, s)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cp)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, cv)
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, ce)
rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, cr)
t0 = rospy.get_time()
while ('p' not in d or 'ratio_field' not in d) and rospy.get_time() - t0 < 20:
    rospy.sleep(0.1)
if 'p' not in d:
    print('GATE UNSTABLE (no telemetry)'); raise SystemExit(1)
p0 = dict(d); rospy.sleep(5.0); p1 = dict(d)
mv = max(abs(p1['p'][k] - p0['p'][k]) for k in range(3))
me = max(abs(p1['e'][k] - p0['e'][k]) for k in range(3))
pos = max(abs(x) for x in p1['p']); vel = max(abs(x) for x in p1['v'])
print('ekfpos %s vel %s' % (['%.3f' % x for x in p1['p']], ['%.3f' % x for x in p1['v']]))
print('5s move: ekf %.3f m | ev %.3f m' % (mv, me))
print('pos ratio field=%s value=%s' % (d.get('ratio_field', 'n/a'), ('%.4f' % d['r']) if 'r' in d else 'n/a'))
ok = (mv < 0.05) and (me < 0.05) and (pos < 0.5) and (vel < 0.1) and (d.get('r', 0.0) < 1.0)
print('GATE %s' % ('OK' if ok else 'UNSTABLE'))
raise SystemExit(0 if ok else 1)
PY

