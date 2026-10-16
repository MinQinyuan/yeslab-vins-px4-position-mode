#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-082: OFFBOARD timeline diagnosis from ulog."""
import sys
from pyulog import ULog

def g(u, n):
    for d in u.data_list:
        if d.name == n:
            return d.data
    return None

for f in sys.argv[1:]:
    u = ULog(f)
    print('=== %s ===' % f.split('/')[-1])
    base = u.start_timestamp / 1e6
    vc = g(u, 'vehicle_control_mode')
    if vc:
        keys = [k for k in vc.keys() if k.startswith('flag_control') and 'enabled' in k]
        print('control_mode keys: %s' % keys)
        prev = None
        for i in range(len(vc['timestamp'])):
            cur = (vc['flag_armed'][i],) + tuple(vc[k][i] for k in keys)
            if cur != prev:
                print('  t=%6.2f armed=%d %s' % (vc['timestamp'][i] / 1e6 - base,
                      cur[0], ' '.join('%s=%d' % (k.replace('flag_control_', '').replace('_enabled', ''), cur[j + 1]) for j, k in enumerate(keys))))
                prev = cur
    for n in ('commander_state', 'vehicle_status', 'actuator_armed'):
        d = g(u, n)
        if not d:
            continue
        key = 'main_state' if n == 'commander_state' else ('nav_state' if n == 'vehicle_status' else 'armed')
        prev = None
        for i in range(len(d['timestamp'])):
            cur = d[key][i]
            if cur != prev:
                print('  %-16s t=%6.2f %s=%s' % (n, d['timestamp'][i] / 1e6 - base, key, cur))
                prev = cur
    oc = g(u, 'offboard_control_mode')
    if oc:
        ts = oc['timestamp']
        print('  offboard_control_mode n=%d t=%.2f..%.2f' % (len(ts), ts[0] / 1e6 - base, ts[-1] / 1e6 - base))
    sp = g(u, 'vehicle_local_position_setpoint')
    if sp:
        print('  position_setpoint n=%d  first(x,y,z)=(%.3f,%.3f,%.3f) last=(%.3f,%.3f,%.3f)' % (
            len(sp['timestamp']), sp['x'][0], sp['y'][0], sp['z'][0], sp['x'][-1], sp['y'][-1], sp['z'][-1]))
        print('  sp z min/max %.3f/%.3f' % (min(sp['z']), max(sp['z'])))
    lp = g(u, 'vehicle_local_position')
    if lp:
        print('  vlp z min/max %.3f/%.3f  t=%.2f..%.2f' % (min(lp['z']), max(lp['z']),
              lp['timestamp'][0] / 1e6 - base, lp['timestamp'][-1] / 1e6 - base))
    cmd = g(u, 'vehicle_command')
    if cmd:
        print('  vehicle_command n=%d:' % len(cmd['timestamp']))
        for i in range(len(cmd['timestamp'])):
            print('    t=%6.2f cmd=%s p1=%.1f p2=%.1f' % (cmd['timestamp'][i] / 1e6 - base, cmd['command'][i], cmd['param1'][i], cmd['param2'][i]))
