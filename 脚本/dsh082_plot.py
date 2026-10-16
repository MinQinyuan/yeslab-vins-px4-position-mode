#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSH-082 Step D figure: px4_local_position.png (from flight-1 CSV)."""
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rows = []
with open('/tmp/dsh082_flight1.csv') as f:
    for r in csv.DictReader(f):
        if not r['lp_x'] or not r['mode']:
            continue
        rows.append(r)
t = [float(r['t']) for r in rows]
def c(k):
    return [float(r[k]) for r in rows]
ph = [r['phase'] for r in rows]
md = [r['mode'] for r in rows]
err = c('err')
vm = [(c('lv_x')[i] ** 2 + c('lv_y')[i] ** 2 + c('lv_z')[i] ** 2) ** 0.5 for i in range(len(rows))]

fig, ax = plt.subplots(4, 1, figsize=(11.5, 12.5), sharex=True)
for i, (k, name) in enumerate([('x', 'X (East)'), ('y', 'Y (North)'), ('z', 'Z (Up)')]):
    a = ax[i]
    a.plot(t, c('lp_' + k), 'b-', lw=1.6, label='PX4 local_position (EKF2, EV-driven)')
    a.plot(t, c('sp_' + k), 'r--', lw=1.3, label='OFFBOARD setpoint (MAVROS)')
    a.plot(t, c('ev_' + k), 'g:', lw=1.0, label='External Vision input (VINS odom)')
    a.set_ylabel(name + ' [m]')
    a.grid(alpha=.3)
    if i == 0:
        a.legend(loc='upper left', fontsize=8)
a = ax[3]
a.plot(t, err, 'k-', lw=1.4, label='|local_position - setpoint| [m]')
a.plot(t, vm, color='darkorange', lw=1.2, label='|velocity| [m/s]')
a.axhline(0.30, color='gray', ls=':', lw=1)
a.set_ylabel('tracking error [m] / |v| [m/s]')
a.set_xlabel('sim time [s]')
a.grid(alpha=.3)
a.legend(loc='upper left', fontsize=8)

# shade OFFBOARD window + phase markers
i0 = next((i for i in range(len(md)) if md[i] == 'OFFBOARD'), None)
i1 = len(md) - 1 - next((i for i in range(len(md)) if md[len(md) - 1 - i] == 'OFFBOARD'), 0)
if i0 is not None:
    for a in ax:
        a.axvspan(t[i0], t[i1], color='gold', alpha=.12)
    ax[0].annotate('OFFBOARD  %.1f-%.1f s' % (t[i0], t[i1]), xy=(t[i0], 0.5), xycoords=('data', 'axes fraction'),
                   fontsize=8, color='darkgoldenrod')
seen = set()
for i, p in enumerate(ph):
    if p not in seen:
        seen.add(p)
        for a in ax:
            a.axvline(t[i], color='gray', lw=.5, alpha=.5)
        ax[0].text(t[i], 1.01, p, rotation=45, fontsize=7, ha='left', va='bottom', transform=ax[0].get_xaxis_transform())
fig.suptitle('DSH-082 Step D - PX4 local position under External Vision (VINS), OFFBOARD small trajectory\n'
             'headless SITL | EKF2_AID_MASK=8, EKF2_HGT_MODE=3 | no GPS fusion', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig('/home/min/px4_local_position.png', dpi=130)
print('saved /home/min/px4_local_position.png  rows=%d' % len(rows))
