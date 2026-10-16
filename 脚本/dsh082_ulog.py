#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-082: compact ulog summary for Step D evidence (EKF validity, EV aid, Offboard setpoints)."""
import sys
from pyulog import ULog

def summ(f):
    u = ULog(f)
    names = {}
    for d in u.data_list:
        names.setdefault(d.name, []).append(d)
    print('=== %s ===' % f.split('/')[-1])
    print('topics(%d): %s' % (len(names), ' '.join(sorted(names.keys()))))
    def g(n):
        ds = names.get(n)
        return ds[0].data if ds else None
    for n in ('trajectory_setpoint', 'offboard_control_mode', 'vehicle_command', 'position_setpoint_triplet'):
        d = g(n)
        print('%-24s %s' % (n, ('%d msgs' % len(d['timestamp'])) if d else 'ABSENT'))
    v = g('vehicle_local_position')
    if v:
        print('vlp n=%d xy_valid=%s z_valid=%s v_xy_valid=%s v_z_valid=%s' % (
            len(v['timestamp']), sorted(set(v['xy_valid'])), sorted(set(v['z_valid'])),
            sorted(set(v['v_xy_valid'])), sorted(set(v['v_z_valid']))))
        print('vlp x %.3f->%.3f  y %.3f->%.3f  z %.3f->%.3f' % (
            v['x'][0], v['x'][-1], v['y'][0], v['y'][-1], v['z'][0], v['z'][-1]))
        vmax = max((v['vx'][i] ** 2 + v['vy'][i] ** 2 + v['vz'][i] ** 2) ** 0.5 for i in range(len(v['vx'])))
        print('vlp z min/max %.3f/%.3f  |v|max %.3f' % (min(v['z']), max(v['z']), vmax))
    e = g('estimator_status')
    if e:
        ks = [k for k in e.keys() if 'ratio' in k]
        print('estimator_status n=%d ratio_keys=%s' % (len(e['timestamp']), ks))
        for k in ks:
            vals = [x for x in e[k] if x == x]
            if vals:
                print('  %-22s min %8.3f max %8.3f last %8.3f' % (k, min(vals), max(vals), vals[-1]))
    sf = g('estimator_status_flags')
    if sf:
        print('estimator_status_flags n=%d' % len(sf['timestamp']))
        for k in sorted(sf.keys()):
            if k.startswith('cs_'):
                s = sorted(set(sf[k]))
                print('  %-24s %s' % (k, s[0] if len(s) == 1 else 'varies%s' % s))
    for n in ('estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt', 'estimator_aid_src_gnss_pos', 'estimator_aid_src_gnss_hgt'):
        d = g(n)
        if d:
            print('%-26s n=%-6d fused=%s' % (n, len(d['timestamp']), sorted(set(d.get('fused', [])))[:4]))
        else:
            print('%-26s ABSENT' % n)
    cs = g('commander_state')
    if cs:
        print('commander_state main_state set=%s' % sorted(set(cs['main_state'])))
    vs = g('vehicle_status')
    if vs:
        print('vehicle_status nav_state=%s arming_state=%s' % (sorted(set(vs['nav_state'])), sorted(set(vs['arming_state']))))
    gs = g('sensor_gps')
    if gs:
        print('sensor_gps n=%d fix_type=%s' % (len(gs['timestamp']), sorted(set(gs.get('fix_type', [])))))

for f in sys.argv[1:]:
    try:
        summ(f)
    except Exception as ex:
        print('ERR %s: %s' % (f, ex))
