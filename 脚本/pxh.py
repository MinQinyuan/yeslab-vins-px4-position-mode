#!/usr/bin/env python3
# pxh.py —— 非交互式 PX4 shell 客户端（经 GCS MAVLink 实例 udpout，不 bind 本地端口）
# 用法: python3 pxh.py [udpout:127.0.0.1:18570] "命令1" "命令2" ...
import sys, time
from pymavlink import mavutil

port = 'udpout:127.0.0.1:18570'
args = sys.argv[1:]
if args and ':' in args[0]:
    port = args[0]; args = args[1:]
cmds = args if args else ['ekf2 status']
dur = float(__import__('os').environ.get('PXH_WAIT', '8'))

m = mavutil.mavlink_connection(port, source_system=250, source_component=190)
try:
    m.wait_heartbeat(timeout=10)
    print("# heartbeat OK (%s)" % port)
except Exception as e:
    print("# NO heartbeat: %s" % e)

DEV = getattr(mavutil.mavlink, 'SERIAL_CONTROL_DEV_SHELL', 10)

def send(cmd):
    data = (cmd + "\n").encode('utf-8')
    buf = list(data) + [0] * (70 - len(data))
    m.mav.serial_control_send(DEV, 0, 0, 0, len(data), buf[:70])

for c in cmds:
    send(c)
    time.sleep(0.6)

t0 = time.time()
while time.time() - t0 < dur:
    msg = m.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=1)
    if msg is None:
        continue
    data = bytes(b for b in msg.data[:msg.count])
    if data:
        sys.stdout.write(data.decode('utf-8', 'replace'))
        sys.stdout.flush()
print()
