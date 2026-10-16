#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-091 悬停测试：AUTO.TAKEOFF -> AUTO.LOITER 保持 30 s -> 切 POSCTL 保持 30 s -> AUTO.LAND
依据 GPT-086 第八节流程 + 第六节守卫阈值；沿用 DSH-087 已验证的守卫实现。
守卫（仅脚本层，不涉 PX4 参数）：
  |Δxy| > 0.60 m                        -> 中止
  |Δz|  > +1.80 m / < -0.35 m           -> 中止（★用户 2026-10-15 批准放宽竖直包线）
  水平速度 > 0.35 m/s 持续 > 0.8 s       -> 中止（垂直爬升期合法超速，故只算水平分量）
  3D 速度 > 1.00 m/s 持续 > 0.5 s        -> 中止
  EV |p| > 1.5 m / VINS |p| > 2.0 m      -> 中止
中止动作：立即 AUTO.LAND + disarm；LAND 阶段同样守卫，异常立即强制上锁(21196)。
摇杆：ManualControl 原始 0~1000 量纲，500 = 中位；POSCTL/TO_POSCTL 段发 z=500 保持高度。
"""
import csv
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode, CommandLong

CSV = '/tmp/dsh095_hover.csv'
STATUS = '/tmp/dsh095_hover_status.txt'
PHASE = '/tmp/dsh095_phase.txt'
TAKEOVER = '/tmp/dsh095_posctl_takeover.txt'

T_ARM_MAX = 150.0
TAKEOFF_DZ = 1.10          # 到达即切 LOITER（任务书 1~1.5 m；取 0.9 m 留漂移余量）
LOITER_HOLD = 30.0         # ★ 通过标准：连续稳定悬停 >= 30 s
POSCTL_HOLD = 30.0         # ★ POSCTL 位置模式稳定悬停 30 s
LIM_XY = 0.60
LIM_Z_UP = 2.20            # ★用户 2026-10-15 批准：竖直包线 +1.80 m（水平仍 0.60 m）
LIM_Z_DN = 0.35            # ★竖直向下包线 0.35 m
LIM_VXY = 0.35
LIM_VXY_SUSTAIN = 0.8
LIM_V3D = 1.00
LIM_V3D_SUSTAIN = 0.5
LIM_EV = 2.20
LIM_VINS = 2.0

rospy.init_node('dsh091_hover', anonymous=True)
st = State()
d = {}


def cbs(m):
    global st
    st = m


def cbp(m):
    d['p'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)


def cbv(m):
    d['v'] = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)


def cbe(m):
    d['e'] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)


def cbo(m):
    d['o'] = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)


rospy.Subscriber('/mavros/state', State, cbs)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cbp)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, cbv)
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, cbe)
rospy.Subscriber('/vins_estimator/odometry', Odometry, cbo)

mc_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
ar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
ms = rospy.ServiceProxy('/mavros/set_mode', SetMode)
cmdl = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

t0 = time.time()
while len(d) < 4 and time.time() - t0 < 20:
    time.sleep(0.1)
if len(d) < 4:
    open(STATUS, 'w').write('FATAL: no telemetry (got %s)\n' % sorted(d.keys()))
    print('FATAL: no telemetry')
    sys.exit(1)

home = tuple(d['p'])
f = open(CSV, 'w', newline='')
w = csv.writer(f)
w.writerow(['t', 'phase', 'lp_x', 'lp_y', 'lp_z', 'lv_x', 'lv_y', 'lv_z',
            'ev_x', 'ev_y', 'ev_z', 'vins_x', 'vins_y', 'vins_z',
            'sx', 'sy', 'sz', 'sr', 'mode', 'armed', 'dxy', 'dxyz'])
tk = open(TAKEOVER, 'w')

events = []
abort = None
t_arm = t_posctl = t_loiter = None
maxdxy = maxdxyz = maxvxy = maxv3d = evdrift = 0.0
ev_at_posctl = None
vxy_over_since = None
v3d_over_since = None
pstats = {}


def rec(phase, sx, sy, sz, sr):
    global maxdxy, maxdxyz, maxvxy, maxv3d
    mc = ManualControl()
    mc.header.stamp = rospy.Time.now()
    mc.x, mc.y, mc.z, mc.r = sx, sy, sz, sr
    mc_pub.publish(mc)
    p = d['p']
    v = d['v']
    e = d['e']
    o = d['o']
    dz = p[2] - home[2]
    dxy = math.hypot(p[0] - home[0], p[1] - home[1])
    dxyz = math.sqrt(sum((p[k] - home[k]) ** 2 for k in range(3)))
    vxy = math.hypot(v[0], v[1])
    v3d = math.sqrt(sum(x * x for x in v))
    maxdxy = max(maxdxy, dxy)
    maxdxyz = max(maxdxyz, dxyz)
    maxvxy = max(maxvxy, vxy)
    maxv3d = max(maxv3d, v3d)
    s = pstats.setdefault(phase, {'dzmin': 1e9, 'dzmax': -1e9, 'dxymax': 0.0, 't0': time.time(), 't1': time.time()})
    s['dzmin'] = min(s['dzmin'], dz)
    s['dzmax'] = max(s['dzmax'], dz)
    s['dxymax'] = max(s['dxymax'], dxy)
    s['t1'] = time.time()
    w.writerow(['%.3f' % (time.time() - t0), phase] + ['%.4f' % x for x in p] +
               ['%.4f' % x for x in v] + ['%.4f' % x for x in e] + ['%.4f' % x for x in o] +
               ['%.1f' % sx, '%.1f' % sy, '%.1f' % sz, '%.1f' % sr, st.mode, st.armed,
                '%.4f' % dxy, '%.4f' % dxyz])
    try:
        open(PHASE, 'w').write('%s t=%.1f armed=%s mode=%s dz=%.3f dxy=%.3f vxy=%.3f\n'
                               % (phase, time.time() - t0, st.armed, st.mode, dz, dxy, vxy))
    except IOError:
        pass
    if phase.startswith('POSCTL'):
        tk.write('%.3f %.4f %.4f %.4f %.4f %.4f %.4f %.1f %.1f\n'
                 % (time.time() - t0, p[0], p[1], p[2], e[0], e[1], e[2], sx, sz))
    return dxy, dxyz, vxy, v3d


def guard(dxy, dxyz, vxy, v3d, phase):
    global vxy_over_since, v3d_over_since
    if not st.armed:
        return None
    if dxy > LIM_XY:
        return 'EKF |dxy|=%.3f m > %.2f @%s' % (dxy, LIM_XY, phase)
    dz = d['p'][2] - home[2]
    if dz > LIM_Z_UP:
        return 'EKF dz=%.3f m > +%.2f @%s' % (dz, LIM_Z_UP, phase)
    if dz < -LIM_Z_DN:
        return 'EKF dz=%.3f m < -%.2f @%s' % (dz, LIM_Z_DN, phase)
    ev = max(abs(x) for x in d['e'])
    if ev > LIM_EV:
        return 'EV |p|=%.3f m > %.2f @%s' % (ev, LIM_EV, phase)
    vn = math.sqrt(sum(x * x for x in d['o']))
    if vn > LIM_VINS:
        return 'VINS |p|=%.3f m > %.2f @%s' % (vn, LIM_VINS, phase)
    now = time.time()
    if vxy > LIM_VXY:
        vxy_over_since = vxy_over_since or now
        if now - vxy_over_since > LIM_VXY_SUSTAIN:
            return 'horiz v=%.3f m/s > %.2f for >%.1fs @%s' % (vxy, LIM_VXY, LIM_VXY_SUSTAIN, phase)
    else:
        vxy_over_since = None
    if v3d > LIM_V3D:
        v3d_over_since = v3d_over_since or now
        if now - v3d_over_since > LIM_V3D_SUSTAIN:
            return '3D v=%.3f m/s > %.2f for >%.1fs @%s' % (v3d, LIM_V3D, LIM_V3D_SUSTAIN, phase)
    else:
        v3d_over_since = None
    return None


def run(phase, dur, sz=0.0, watch=True):
    ps = time.time()
    while time.time() - ps < dur:
        dxy, dxyz, vxy, v3d = rec(phase, 0, 0, sz, 0)
        if watch:
            a = guard(dxy, dxyz, vxy, v3d, phase)
            if a:
                return a
        time.sleep(0.05)
    return None


# ---------- PRE + ARM (AUTO.LOITER) ----------
run('PRE', 1.5, watch=False)
ps = time.time()
last = 0.0
while time.time() - ps < 8.0:
    rec('ARM', 0, 0, 0, 0)
    if st.armed:
        t_arm = time.time()
        events.append('ARMED at t=%.1f' % (t_arm - t0))
        break
    if time.time() - last > 1.2:
        last = time.time()
        try:
            if st.mode != 'AUTO.LOITER':
                r = ms(custom_mode='AUTO.LOITER')
                events.append('AUTO.LOITER sent=%s' % r.mode_sent)
            else:
                r = ar(True)
                events.append('arm success=%s' % r.success)
        except rospy.ServiceException as ex:
            events.append('arm srv err %s' % ex)
    time.sleep(0.05)
if not st.armed:
    abort = 'could not arm'
else:
    t_arm = time.time()

# ---------- AUTO.TAKEOFF ----------
if not abort:
    try:
        r = ms(custom_mode='AUTO.TAKEOFF')
        events.append('AUTO.TAKEOFF sent=%s' % r.mode_sent)
    except rospy.ServiceException as ex:
        abort = 'AUTO.TAKEOFF srv err %s' % ex
if not abort:
    ps = time.time()
    while time.time() - ps < 25.0 and time.time() - t_arm < T_ARM_MAX:
        dxy, dxyz, vxy, v3d = rec('TAKEOFF', 0, 0, 0, 0)
        a = guard(dxy, dxyz, vxy, v3d, 'TAKEOFF')
        if a:
            abort = a
            break
        if d['p'][2] - home[2] >= TAKEOFF_DZ:
            events.append('dz=%.3f m at t_arm+%.1fs' % (d['p'][2] - home[2], time.time() - t_arm))
            break
        time.sleep(0.05)
    if not abort and d['p'][2] - home[2] < TAKEOFF_DZ:
        abort = 'AUTO.TAKEOFF did not climb (dz=%.3f mode=%s)' % (d['p'][2] - home[2], st.mode)

# ---------- AUTO.LOITER 悬停 30 s ----------
if not abort:
    ps = time.time()
    while time.time() - ps < 3.0 and st.mode != 'AUTO.LOITER':
        rec('TO_LOITER', 0, 0, 0, 0)
        try:
            r = ms(custom_mode='AUTO.LOITER')
            events.append('AUTO.LOITER sent=%s' % r.mode_sent)
        except rospy.ServiceException as ex:
            events.append('LOITER err %s' % ex)
        time.sleep(0.3)
    if st.mode != 'AUTO.LOITER':
        abort = 'AUTO.LOITER not entered (mode=%s)' % st.mode
    else:
        t_loiter = time.time()
        events.append('IN AUTO.LOITER at t_arm+%.1fs dz=%.3f'
                      % (t_loiter - t_arm, d['p'][2] - home[2]))
if not abort:
    ps = time.time()
    while time.time() - ps < LOITER_HOLD and time.time() - t_arm < T_ARM_MAX:
        dz = d['p'][2] - home[2]
        sz = 0.0
        if dz < 0.20:
            sz = 650.0          # 仅防沉地，轻微抬升
        dxy, dxyz, vxy, v3d = rec('LOITER', 0, 0, sz, 0)
        a = guard(dxy, dxyz, vxy, v3d, 'LOITER')
        if a:
            abort = a
            break
        time.sleep(0.05)
    if not abort:
        events.append('LOITER hold done %.1fs dz=%.3f dxy=%.3f'
                      % (time.time() - t_loiter, d['p'][2] - home[2],
                         math.hypot(d['p'][0] - home[0], d['p'][1] - home[1])))

# ---------- 切 POSCTL ----------
if not abort:
    ps = time.time()
    while time.time() - ps < 3.0 and st.mode != 'POSCTL':
        rec('TO_POSCTL', 0, 0, 500, 0)
        try:
            r = ms(custom_mode='POSCTL')
            events.append('POSCTL sent=%s' % r.mode_sent)
        except rospy.ServiceException as ex:
            events.append('POSCTL err %s' % ex)
        time.sleep(0.2)
    if st.mode != 'POSCTL':
        abort = 'POSCTL not entered (mode=%s)' % st.mode
    else:
        t_posctl = time.time()
        ev_at_posctl = tuple(d['e'])
        events.append('IN POSCTL at t_arm+%.1fs dz=%.3f dxy=%.3f'
                      % (t_posctl - t_arm, d['p'][2] - home[2],
                         math.hypot(d['p'][0] - home[0], d['p'][1] - home[1])))

# ---------- POSCTL 悬停 30 s ----------
if not abort:
    ps = time.time()
    while time.time() - ps < POSCTL_HOLD and time.time() - t_arm < T_ARM_MAX:
        dxy, dxyz, vxy, v3d = rec('POSCTL', 0, 0, 500, 0)
        a = guard(dxy, dxyz, vxy, v3d, 'POSCTL')
        if a:
            abort = a
            break
        time.sleep(0.05)
    if not abort:
        events.append('POSCTL hold done %.1fs dz=%.3f dxy=%.3f'
                      % (time.time() - ps, d['p'][2] - home[2],
                         math.hypot(d['p'][0] - home[0], d['p'][1] - home[1])))
tk.close()

# ---------- 结束：AUTO.LAND + disarm ----------
try:
    r = ms(custom_mode='AUTO.LAND')
    events.append('AUTO.LAND sent=%s' % r.mode_sent)
except rospy.ServiceException as ex:
    events.append('AUTO.LAND err %s' % ex)
ps = time.time()
while st.armed and time.time() - ps < 20.0:
    dxy, dxyz, vxy, v3d = rec('LAND', 0, 0, 0, 0)
    if (d['p'][2] - home[2]) > 2.2 or v3d > 1.2:      # ★ LAND 阶段也保持守卫，异常立即强制上锁
        events.append('LAND runaway dxyz=%.2f v3d=%.2f -> immediate force disarm' % (dxyz, v3d))
        break
    time.sleep(0.05)
if st.armed:
    try:
        r = ar(False)
        events.append('disarm success=%s' % r.success)
    except rospy.ServiceException as ex:
        events.append('disarm err %s' % ex)
    time.sleep(1.0)
if st.armed:
    try:
        r = cmdl(broadcast=False, command=400, confirmation=0, param1=0.0, param2=21196.0,
                 param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
        events.append('FORCE disarm(21196) success=%s' % r.success)
    except rospy.ServiceException as ex:
        events.append('force disarm err %s' % ex)
f.close()

t_end = time.time()
if ev_at_posctl is not None:
    evdrift = math.hypot(d['e'][0] - ev_at_posctl[0], d['e'][1] - ev_at_posctl[1])
try:
    vins_ep = float(open('/tmp/dsh087_vins_restart2_epoch').read().strip())
    vins_age = t_arm - vins_ep
except Exception:
    vins_age = -1.0

out = ['RESULT: %s' % ('ABORT: ' + abort if abort else 'COMPLETED'),
       'armed window: %.1f s   VINS age at arm(2nd restart): %.1f s' % (t_end - t_arm, vins_age),
       'END mode=%s armed=%s' % (st.mode, st.armed),
       'max |dxy|=%.3f m  max |dxyz|=%.3f m  max vxy=%.3f  max v3d=%.3f m/s' % (maxdxy, maxdxyz, maxvxy, maxv3d),
       'POSCTL-window EV drift(xy)=%.3f m' % evdrift,
       'LOITER entered=%s  POSCTL entered=%s' % (t_loiter is not None, t_posctl is not None)]
for ph in ('TAKEOFF', 'LOITER', 'POSCTL', 'LAND'):
    s = pstats.get(ph)
    if s:
        out.append('  %-8s dur=%5.1fs  dz %.3f..%.3f (range %.3f m)  max dxy=%.3f m'
                   % (ph, s['t1'] - s['t0'], s['dzmin'], s['dzmax'],
                      s['dzmax'] - s['dzmin'], s['dxymax']))
out += ['EVT ' + e for e in events]
open(STATUS, 'w').write('\n'.join(out) + '\n')
print('\n'.join(out))
print('CSV -> ' + CSV)
