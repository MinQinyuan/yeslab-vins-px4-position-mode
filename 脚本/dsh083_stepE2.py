#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-083 Step E v2: POSCTL hover + small move with CLOSED-LOOP stick scaling.
MAVLink MANUAL_CONTROL fields are raw (int16-scale) in mavros: 0.45 truncates to 0.
So sticks are sent as raw magnitude and the scale is auto-adapted from EKF altitude feedback.
No position injection, no ground truth. Single run, <=60 s (GPT-083 §7).
"""
import rospy, csv, math, sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode

CSV = '/tmp/dsh083_stepE.csv'
STATUS = '/tmp/dsh083_stepE_status.txt'
rospy.init_node('dsh083_stepE2', anonymous=True)
st = State(); d = {}
def cbs(m):
    global st; st = m
def cbp(m): d['p'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
def cbv(m): d['v'] = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)
def cbe(m): d['e'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
rospy.Subscriber('/mavros/state', State, cbs)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cbp)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, cbv)
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, cbe)
mc_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
ar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
ms = rospy.ServiceProxy('/mavros/set_mode', SetMode)

t0 = rospy.get_time()
while len(d) < 3 and rospy.get_time() - t0 < 20:
    rospy.sleep(0.1)
if len(d) < 3:
    print('FATAL: no telemetry'); sys.exit(1)
home = tuple(d['p'])
print('START local %.3f %.3f %.3f armed=%s mode=%s' % (home + (st.armed, st.mode)))

f = open(CSV, 'w', newline='')
w = csv.writer(f)
w.writerow(['t', 'phase', 'lp_x', 'lp_y', 'lp_z', 'lv_x', 'lv_y', 'lv_z', 'ev_x', 'ev_y', 'ev_z',
            'sx', 'sy', 'sz', 'sr', 'mode', 'armed', 'dist', 'dz'])
events = []; abort = None; maxdist = 0.0; maxvm = 0.0; maxdz = 0.0
S = [150.0]
def emit(phase, sx, sy, sz, sr):
    global maxdist, maxvm, maxdz
    mc = ManualControl()
    mc.header.stamp = rospy.Time.now()
    mc.x, mc.y, mc.z, mc.r = sx, sy, sz, sr
    mc_pub.publish(mc)
    p = d['p']; v = d['v']; e = d['e']
    dist = math.sqrt(sum((p[k] - home[k]) ** 2 for k in range(3)))
    dz = p[2] - home[2]
    vm = math.sqrt(sum(x * x for x in v))
    maxdist = max(maxdist, dist); maxvm = max(maxvm, vm); maxdz = max(maxdz, dz)
    w.writerow(['%.3f' % (rospy.get_time() - t0), phase] + ['%.4f' % x for x in p] + ['%.4f' % x for x in v] +
               ['%.4f' % x for x in e] + ['%.1f' % sx, '%.1f' % sy, '%.1f' % sz, '%.1f' % sr,
                st.mode, st.armed, '%.3f' % dist, '%.3f' % dz])

def hold(phase, dur, sx=0.0, sy=0.0, sz=0.0, sr=0.0):
    ps = rospy.get_time()
    while rospy.get_time() - ps < dur:
        emit(phase, sx, sy, sz, sr)
        rospy.sleep(0.05)

# ---- PRE / ARM (LOITER arm, proven) ----
hold('PRE', 3.0)
ps = rospy.get_time(); last = 0.0
while rospy.get_time() - ps < 14.0:
    emit('ARM', 0, 0, 0, 0)
    if rospy.get_time() - last > 1.2:
        last = rospy.get_time()
        try:
            if not st.armed:
                if st.mode != 'AUTO.LOITER':
                    r = ms(custom_mode='AUTO.LOITER'); events.append('LOITER sent=%s' % r.mode_sent)
                else:
                    r = ar(True); events.append('arm success=%s' % r.success)
            else:
                events.append('ARMED (mode %s)' % st.mode); break
        except rospy.ServiceException as e:
            events.append('srv err %s' % e)
    rospy.sleep(0.05)
if not st.armed:
    abort = 'could not arm'; 
# ---- POSCTL ----
ps = rospy.get_time()
while st.armed and rospy.get_time() - ps < 5.0:
    emit('POSCTL', 0, 0, 0, 0)
    if st.mode == 'POSCTL':
        events.append('IN POSCTL'); break
    if rospy.get_time() - last > 1.0:
        last = rospy.get_time()
        try:
            r = ms(custom_mode='POSCTL'); events.append('POSCTL sent=%s' % r.mode_sent)
        except rospy.ServiceException as e:
            events.append('POSCTL err %s' % e)
    rospy.sleep(0.05)
if not abort and st.mode != 'POSCTL':
    abort = 'POSCTL not entered (mode %s)' % st.mode

# ---- CLIMB closed loop: raw stick, escalate once if no response, stop at +0.30 m ----
if not abort:
    ps = rospy.get_time()
    while rospy.get_time() - ps < 9.0:
        dz = d['p'][2] - home[2]
        if dz >= 0.30:
            events.append('climbed to %.2f m with stick raw %.0f' % (dz, S[0])); break
        if rospy.get_time() - ps > 2.5 and dz < 0.03 and S[0] == 150.0:
            S[0] = 800.0; events.append('no response at raw 150 -> escalate to raw 800')
        emit('CLIMB', 0, 0, S[0], 0)
        rospy.sleep(0.05)
    if d['p'][2] - home[2] < 0.05:
        abort = 'no climb (dz=%.3f, stick raw %.0f)' % (d['p'][2] - home[2], S[0])
if not abort:
    # ---- HOVER 12 s (centered sticks = position/altitude hold in POSCTL) ----
    hold('HOVER', 12.0)
    L = 150.0 if S[0] == 150.0 else 300.0
    # ---- small moves in POSCTL ----
    hold('MX', 3.0, sx=L)
    hold('HOLDX', 3.0)
    hold('MY', 3.0, sy=L)
    hold('HOLDY', 3.0)
    hold('BACK', 5.0, sx=-L, sy=-L)
    hold('HOLDB', 2.0)
    # ---- descend with closed loop ----
    ps = rospy.get_time()
    while rospy.get_time() - ps < 8.0 and st.armed:
        if d['p'][2] - home[2] <= 0.06:
            events.append('descended to %.2f m' % (d['p'][2] - home[2])); break
        emit('DESC', 0, 0, -S[0], 0)
        rospy.sleep(0.05)
# ---- LAND ----
ps = rospy.get_time()
try:
    ms(custom_mode='AUTO.LAND'); events.append('AUTO.LAND sent')
except rospy.ServiceException as e:
    events.append('land err %s' % e)
while rospy.get_time() - ps < 15.0 and st.armed:
    emit('LAND', 0, 0, 0, 0)
    rospy.sleep(0.05)
try:
    if st.armed:
        ar(False); events.append('disarm commanded')
except rospy.ServiceException:
    pass
f.close()
out = ['ABORT: %s' % (abort if abort else 'none'),
       'END mode=%s armed=%s' % (st.mode, st.armed),
       'max dist=%.3f m  max dz=%.3f m  max |v|=%.3f m/s  stick scale=%s' % (maxdist, maxdz, maxvm, S[0])]
out += ['EVT ' + e for e in events]
open(STATUS, 'w').write('\n'.join(out) + '\n')
print('\n'.join(out))
print('CSV -> ' + CSV)
