#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-082: Step D flight-window ulog table (2 s bins) + EV drift check."""
import sys
from pyulog import ULog

f = sys.argv[1]
u = ULog(f)
def g(n):
    for d in u.data_list:
        if d.name == n:
            return d.data
    return None
base = u.start_timestamp / 1e6
vlp = g('vehicle_local_position'); est = g('estimator_status')
ld = g('vehicle_land_detected'); ev = g('vehicle_visual_odometry')
ac = g('actuator_controls_0'); sf = g('estimator_status_flags')
print('log %s  dur %.1f s' % (f.split('/')[-1], vlp['timestamp'][-1] / 1e6 - base))
print('vlp xy_valid=%s z_valid=%s v_xy_valid=%s v_z_valid=%s' % (
    sorted(set(vlp['xy_valid'])), sorted(set(vlp['z_valid'])), sorted(set(vlp['v_xy_valid'])), sorted(set(vlp['v_z_valid']))))
print('vlp x %.3f->%.3f  y %.3f->%.3f  z %.3f->%.3f' % (
    vlp['x'][0], vlp['x'][-1], vlp['y'][0], vlp['y'][-1], vlp['z'][0], vlp['z'][-1]))
for k in ('pos_test_ratio', 'hgt_test_ratio', 'vel_test_ratio'):
    vals = [v for v in est[k] if v == v]
    print('%-16s min %.4f max %.4f last %.4f' % (k, min(vals), max(vals), vals[-1]))
if sf:
    for k in ('cs_ev_pos', 'cs_ev_hgt', 'cs_ev_vel', 'cs_gps', 'cs_gps_hgt', 'cs_baro_hgt', 'cs_in_air'):
        if k in sf:
            print('  %-12s %s' % (k, sorted(set(sf[k]))))
if ld:
    for k in ('landed', 'freefall', 'ground_contact'):
        if k in ld:
            t = ld['timestamp']
            prev = None
            tr = []
            for i in range(len(t)):
                cur = ld[k][i]
                if cur != prev:
                    tr.append('%.1f:%s' % (t[i] / 1e6 - base, cur))
                    prev = cur
            print('land_detected %-15s %s' % (k, ' '.join(tr[:12])))
if ev:
    print('EV n=%d  x %.3f->%.3f y %.3f->%.3f z %.3f->%.3f' % (len(ev['timestamp']),
          ev['x'][0], ev['x'][-1], ev['y'][0], ev['y'][-1], ev['z'][0], ev['z'][-1]))
print()
print('   t   vlp_x  vlp_y  vlp_z  vx     vy     vz   |pos|  thr  pos_r  hgt_r  landed')
t0 = vlp['timestamp'][0] / 1e6
for b in range(0, int(vlp['timestamp'][-1] / 1e6 - base) + 1, 3):
    tt = base + b
    i = min(range(len(vlp['timestamp'])), key=lambda j: abs(vlp['timestamp'][j] / 1e6 - tt))
    def at(d, key='timestamp'):
        if d is None:
            return float('nan')
        j = min(range(len(d[key])), key=lambda q: abs(d[key][q] / 1e6 - tt))
        return j
    je = at(est); jl = at(ld); ja = at(ac)
    print('%4d %7.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %5.2f %5.3f %6.3f %d' % (
        b, vlp['x'][i], vlp['y'][i], vlp['z'][i], vlp['vx'][i], vlp['vy'][i], vlp['vz'][i],
        (vlp['x'][i] ** 2 + vlp['y'][i] ** 2 + vlp['z'][i] ** 2) ** 0.5,
        ac['control[3]'][ja] if ac else float('nan'), est['pos_test_ratio'][je], est['hgt_test_ratio'][je],
        ld['landed'][jl] if ld else -1))
