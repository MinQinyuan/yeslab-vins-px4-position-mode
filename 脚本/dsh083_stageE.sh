#!/bin/bash
# DSH-083 Step E orchestrator (guest):  gate -> arm self-test -> evidence terminal (front) -> POSCTL run
source /opt/ros/noetic/setup.bash 2>/dev/null
O=/tmp/dsh083_stageE.txt; : >"$O"
echo "==== [1] EV health gate (GPT-083 §6) ====" >>"$O"
bash /home/min/vins_restart_and_gate.sh >>"$O" 2>&1
G=$(grep -c 'GATE OK' /tmp/dsh083_stageE.txt)
if [ "$G" != "1" ]; then
  echo "==== GATE NOT OK -> Step E aborted, no arm ====" >>"$O"
  exit 2
fi
echo "==== [2] arm self-test (LOITER arm -> report -> disarm) ====" >>"$O"
timeout 40 python3 - <<'PY' >>"$O" 2>&1
import rospy
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
rospy.init_node('dsh083_armtest', anonymous=True)
st=[State()]
rospy.Subscriber('/mavros/state', State, lambda m: st.__setitem__(0,m))
mc=rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
ar=rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
ms=rospy.ServiceProxy('/mavros/set_mode', SetMode)
rospy.sleep(2)
try:
    print('mode -> AUTO.LOITER', ms(custom_mode='AUTO.LOITER').mode_sent)
except rospy.ServiceException as e:
    print('setmode err', e)
ok=False
for i in range(6):
    m=ManualControl(); m.header.stamp=rospy.Time.now(); mc.publish(m)
    try:
        r=ar(True); print('arm try %d success=%s' % (i, r.success))
        if r.success: ok=True; break
    except rospy.ServiceException as e:
        print('arm err', e)
    rospy.sleep(2)
rospy.sleep(2)
print('armed state = %s (mode %s)' % (st[0].armed, st[0].mode))
if st[0].armed:
    try: print('disarm ->', ar(False).success)
    except rospy.ServiceException as e: print('disarm err', e)
    rospy.sleep(3)
print('ARM SELF-TEST %s' % ('PASS' if ok else 'FAIL'))
PY
A=$(grep -c 'ARM SELF-TEST PASS' /tmp/dsh083_stageE.txt)
if [ "$A" != "1" ]; then
  echo "==== ARM SELF-TEST FAILED -> Step E aborted before flight ====" >>"$O"
  exit 3
fi
echo "==== [3] open evidence terminal (front) ====" >>"$O"
T=""
for t in xterm gnome-terminal xfce4-terminal konsole; do command -v $t >/dev/null 2>&1 && T=$t && break; done
echo "terminal=$T  DISPLAY=$DISPLAY  tools: wmctrl=$(command -v wmctrl) xdotool=$(command -v xdotool)" >>"$O"
case $T in
  xterm) DISPLAY=:0 nohup xterm -fa Monospace -fs 12 -geometry 120x32 -T 'DSH-083 Step E POSCTL' -e bash /home/min/dsh083_readout.sh >/dev/null 2>&1 & ;;
  gnome-terminal) DISPLAY=:0 nohup gnome-terminal --title='DSH-083 Step E POSCTL' --geometry=120x32 -- bash /home/min/dsh083_readout.sh >/dev/null 2>&1 & ;;
  xfce4-terminal) DISPLAY=:0 nohup xfce4-terminal -T 'DSH-083 Step E POSCTL' -x bash /home/min/dsh083_readout.sh >/dev/null 2>&1 & ;;
  konsole) DISPLAY=:0 nohup konsole -p tabtitle='DSH-083 Step E POSCTL' -e bash /home/min/dsh083_readout.sh >/dev/null 2>&1 & ;;
  *) echo "no terminal found" >>"$O" ;;
esac
sleep 5
if command -v wmctrl >/dev/null 2>&1; then DISPLAY=:0 wmctrl -a 'DSH-083' >>"$O" 2>&1; DISPLAY=:0 wmctrl -l >>"$O" 2>&1; fi
if command -v xdotool >/dev/null 2>&1; then DISPLAY=:0 xdotool search --name 'DSH-083' windowactivate >>"$O" 2>&1; fi
echo "==== [4] launch POSCTL run in background ====" >>"$O"
nohup python3 /home/min/dsh083_stepE3.py > /home/min/dsh083_stepE.log 2>&1 &
echo "flight pid=$! started $(date)" >>"$O"
echo "==== orchestrator done ====" >>"$O"
