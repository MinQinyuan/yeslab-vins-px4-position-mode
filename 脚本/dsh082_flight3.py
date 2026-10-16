#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-082 Step D v3: MANUAL_CONTROL heartbeat (centered, no positional info) to clear the
'no manual control stick input' failsafe, then OFFBOARD small trajectory.
Evidence only from onboard estimate (MAVROS local_position / vision_pose / setpoint / estimator_status).
"""
import rospy, csv, math, sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ManualControl, EstimatorStatus
from mavros_msgs.srv import CommandBool, SetMode

CSV = '/tmp/dsh082_flight3.csv'
rospy.init_node('dsh082_flight3', anonymous=True)
st = State()
est = {}
lp = {}; lv = {}; ev = {}
def cb_st(m):
    global st; st = m
def cb_est(m): est['h'] = m.pos_horiz_ratio; est['v'] = m.pos_vert_ratio; est['vl'] = m.vel_ratio
def cb_lp(m): lp['p'] = m.pose.position; lp['o'] = m.pose.orientation
def cb_lv(m): lv['v'] = m.twist.linear
def cb_ev(m): ev['p'] = m.pose.position
rospy.Subscriber('/mavros/state', State, cb_st)
rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, cb_est)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_lp)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, cb_lv)
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, cb_ev)
sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
mc_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

t0 = rospy.get_time()
while ('p' not in lp or 'v' not in lv or 'p' not in ev) and rospy.get_time() - t0 < 25:
    rospy.sleep(0.1)
if 'p' not in lp:
    print('FATAL: no /mavros/local_position/pose'); sys.exit(1)
hx, hy, hz = lp['p'].x, lp['p'].y, lp['p'].z
print('START local %.3f %.3f %.3f | mode %s armed %s' % (hx, hy, hz, st.mode, st.armed))
try:
    mode_srv(custom_mode='AUTO.LOITER')
except rospy.ServiceException:
    pass

WP = [('PRE',    4.0,  (hx, hy, hz), 0),
      ('ARM',   14.0,  (hx, hy, hz), 0),
      ('HOLD0',  1.5,  (hx, hy, hz), 0),
      ('CLIMB',  3.0,  (hx, hy, hz + 0.30), 0),
      ('HOLD1',  5.0,  (hx, hy, hz + 0.30), 0),
      ('XP',     4.0,  (hx + 0.30, hy, hz + 0.30), 0),
      ('HOLD2',  4.0,  (hx + 0.30, hy, hz + 0.30), 0),
      ('YP',     4.0,  (hx + 0.30, hy + 0.30, hz + 0.30), 0),
      ('HOLD3',  4.0,  (hx + 0.30, hy + 0.30, hz + 0.30), 0),
      ('RET',    5.0,  (hx, hy, hz + 0.30), 0),
      ('DESC',   3.0,  (hx, hy, hz), 0),
      ('LAND',  30.0,  None, 1)]
LIM_ERR = 2.5
LIM_VEL = 2.5

f = open(CSV, 'w', newline='')
w = csv.writer(f)
w.writerow(['t', 'phase', 'lp_x', 'lp_y', 'lp_z', 'lv_x', 'lv_y', 'lv_z', 'ev_x', 'ev_y', 'ev_z',
            'sp_x', 'sp_y', 'sp_z', 'err', 'mode', 'armed', 'pos_h', 'pos_v', 'vel_r'])
prev = (hx, hy, hz)
abort = None
evlog = []
last_act = 0.0
AIRBORNE = ('CLIMB', 'HOLD1', 'XP', 'HOLD2', 'YP', 'HOLD3', 'RET', 'DESC')

for name, dur, tgt, is_land in WP:
    ps = rospy.get_time()
    sp_t = prev
    while True:
        tn = rospy.get_time()
        el = tn - ps
        if el >= dur:
            break
        if tgt is not None:
            a = min(1.0, el / max(dur * 0.7, 0.001))
            sp = tuple(prev[k] + (tgt[k] - prev[k]) * a for k in range(3))
            sp_t = sp
        else:
            sp = sp_t
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = sp
        if 'o' in lp:
            msg.pose.orientation = lp['o']
        sp_pub.publish(msg)
        mc = ManualControl()
        mc.header.stamp = rospy.Time.now()
        mc_pub.publish(mc)

        if name == 'ARM' and tn - last_act > 1.0:
            last_act = tn
            try:
                if not st.armed:
                    if st.mode != 'AUTO.LOITER':
                        r = mode_srv(custom_mode='AUTO.LOITER')
                        evlog.append('%.1f -> AUTO.LOITER sent=%s' % (el, r.mode_sent))
                    else:
                        r2 = arm_srv(True)
                        evlog.append('%.1f arm success=%s (mode %s)' % (el, r2.success, st.mode))
                else:
                    if st.mode != 'OFFBOARD':
                        r = mode_srv(custom_mode='OFFBOARD')
                        evlog.append('%.1f armed -> OFFBOARD sent=%s' % (el, r.mode_sent))
                    else:
                        evlog.append('%.1f READY armed+OFFBOARD' % el)
                        break
            except rospy.ServiceException as e:
                evlog.append('%.1f srv err %s' % (el, e))
        if name == 'LAND':
            if 0.9 <= el < 1.1:
                try:
                    r = mode_srv(custom_mode='AUTO.LAND')
                    evlog.append('%.1f -> AUTO.LAND sent=%s' % (el, r.mode_sent))
                except rospy.ServiceException as e:
                    evlog.append('%.1f land err %s' % (el, e))
            if not st.armed and el > 3.0:
                evlog.append('%.1f disarmed (landed)' % el)
                break

        p = lp['p']; v = lv['v']; e = ev['p']
        err = math.sqrt((p.x - sp[0]) ** 2 + (p.y - sp[1]) ** 2 + (p.z - sp[2]) ** 2)
        vm = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        w.writerow(['%.3f' % (tn - t0), name, '%.4f' % p.x, '%.4f' % p.y, '%.4f' % p.z,
                    '%.4f' % v.x, '%.4f' % v.y, '%.4f' % v.z,
                    '%.4f' % e.x, '%.4f' % e.y, '%.4f' % e.z,
                    '%.4f' % sp[0], '%.4f' % sp[1], '%.4f' % sp[2], '%.4f' % err,
                    st.mode, st.armed, '%.3f' % est.get('h', -1), '%.3f' % est.get('v', -1), '%.3f' % est.get('vl', -1)])
        if name in AIRBORNE and st.armed and st.mode != 'OFFBOARD':
            abort = 'mode lost (%s) at t=%.1f' % (st.mode, tn - t0)
            break
        if name in AIRBORNE and (err > LIM_ERR or vm > LIM_VEL):
            abort = '%s err=%.2f vel=%.2f at t=%.1f' % (name, err, vm, tn - t0)
            break
        rospy.sleep(0.05)
    w.writerow(['%.3f' % (rospy.get_time() - t0), name + '_END', '', '', '', '', '', '', '', '', '', '', '', '', '', st.mode, st.armed, '', '', ''])
    f.flush()
    if name != 'LAND':
        prev = tgt
    if abort:
        break

if abort:
    print('ABORT: ' + abort)
    try:
        mode_srv(custom_mode='AUTO.LAND')
    except rospy.ServiceException:
        pass
    rospy.sleep(6)
try:
    if st.armed:
        arm_srv(False)
        print('disarmed at end')
except rospy.ServiceException:
    pass
f.close()
print('END mode %s armed %s' % (st.mode, st.armed))
for l in evlog:
    print('EVT ' + l)
print('CSV -> ' + CSV)
