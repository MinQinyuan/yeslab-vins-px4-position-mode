import glob, os
from pyulog import ULog
L='/home/min/yeslab_build/PX4-Autopilot/build/px4_sitl_default/tmp/log/'
f=sorted(glob.glob(L+'*/*.ulg'), key=os.path.getmtime)[-1]
print('log', f, '%.2f MB' % (os.path.getsize(f)/1e6))
u=ULog(f); base=u.start_timestamp/1e6
def g(n):
    for d in u.data_list:
        if d.name==n: return d.data
    return None
mc=g('manual_control_setpoint')
if mc:
    print('mc n=%d  z set=%s  x set=%s' % (len(mc['timestamp']), sorted(set('%.3f'%v for v in mc['z']))[:6], sorted(set('%.3f'%v for v in mc['x']))[:6]))
    print('  z min %.3f max %.3f | x min %.3f max %.3f' % (min(mc['z']),max(mc['z']),min(mc['x']),max(mc['x'])))
    for k in ('valid','data_source','sticks_moving'):
        if k in mc: print('  %s set=%s' % (k, sorted(set(mc[k]))[:6]))
    print('  samples t/z: ' + ' '.join('%.1f:%.3f' % (mc['timestamp'][i]/1e6-base, mc['z'][i]) for i in range(0,len(mc['timestamp']),max(1,len(mc['timestamp'])//14))))
ac=g('actuator_controls_0')
if ac: print('throttle control[3] min %.3f max %.3f last %.3f' % (min(ac['control[3]']),max(ac['control[3]']),ac['control[3]'][-1]))
lp=g('vehicle_local_position')
if lp: print('vlp z min %.3f max %.3f  |v| max %.3f' % (min(lp['z']),max(lp['z']),max((lp['vx'][i]**2+lp['vy'][i]**2+lp['vz'][i]**2)**0.5 for i in range(len(lp['vx'])))))
ld=g('vehicle_land_detected')
if ld:
    t=ld['timestamp']; prev=None; tr=[]
    for i in range(len(t)):
        if ld['landed'][i]!=prev: tr.append('%.1f:%s'%(t[i]/1e6-base,ld['landed'][i])); prev=ld['landed'][i]
    print('landed:', ' '.join(tr[:8]))
cs=g('commander_state')
if cs: print('commander states=%s' % sorted(set(cs['main_state'])))
