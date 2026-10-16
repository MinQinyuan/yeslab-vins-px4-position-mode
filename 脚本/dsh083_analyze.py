#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-083 Step E analyzer: POSCTL hover stability + small-move response from the 20 Hz CSV."""
import sys
import pandas as pd, numpy as np

p = sys.argv[1] if len(sys.argv) > 1 else r'D:\YESLab_WS\vmrun_tmp\dsh083_stepE.csv'
d = pd.read_csv(p)
num = ['lp_x', 'lp_y', 'lp_z', 'lv_x', 'lv_y', 'lv_z', 'ev_x', 'ev_y', 'ev_z', 'dist', 'vm', 'sx', 'sy', 'sz']
for c in num:
    d[c] = pd.to_numeric(d[c], errors='coerce')
d['vm'] = pd.to_numeric(d['vm'], errors='coerce')
x0, y0, z0 = d.lp_x.iloc[0], d.lp_y.iloc[0], d.lp_z.iloc[0]
d['dx'] = d.lp_x - x0; d['dy'] = d.lp_y - y0; d['dz'] = d.lp_z - z0
print('rows=%d  t=%.1f..%.1f s' % (len(d), d.t.min(), d.t.max()))
g = d.groupby('phase', sort=False).agg(n=('t', 'size'), t0=('t', 'min'), t1=('t', 'max'),
                                       maxvm=('vm', 'max'), maxtrack=('dist', 'max'),
                                       dx0=('dx', 'first'), dx1=('dx', 'last'),
                                       dy0=('dy', 'first'), dy1=('dy', 'last'),
                                       dz0=('dz', 'first'), dz1=('dz', 'last'))
print(g.round(3).to_string())
for ph in ('HOVER', 'MX', 'MY', 'BACK'):
    s = d[d.phase == ph]
    if len(s):
        print('%-6s: dx %.3f..%.3f  dy %.3f..%.3f  |v|mean %.3f max %.3f  mode=%s' % (
            ph, s.dx.min(), s.dx.max(), s.dy.min(), s.dy.max(), s.vm.mean(), s.vm.max(), s['mode'].iloc[-1]))
modes = d.groupby('mode').size().to_dict()
print('modes seen:', modes)
print('armed rows: %d / %d' % ((d.armed == True).sum(), len(d)))
print('max |lp-ev| delta: %.3f m' % np.max(np.sqrt((d.lp_x - d.ev_x) ** 2 + (d.lp_y - d.ev_y) ** 2 + (d.lp_z - d.ev_z) ** 2)))
