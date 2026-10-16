!/bin/bash
# DSH-106  应用用户批准的三项修改（每一项都先备份，零删除）
#   [A] VINS 外参 body_T_cam0/1 按 SDF 几何修正：t=(0.2,±0.035,0.035)，光轴=机体 +X
#   [B] 飞行脚本 dsh095_hover.py：3 处 bug + 守卫按「1~1.5 m 悬停」重标定
#   [C] 新建高对比度特征世界 vins_features.world（empty.world 原封不动）
#   [D] 冷启动脚本改用新世界
#   [E] 打印双目相机内参文件供核对
OUT=/home/min/dsh106_out.txt
exec > "$OUT" 2>&1
date -u '+==== DSH-106 apply approved changes %F %T ===='

BASE=/home/min/yeslab_build
PX4="$BASE/PX4-Autopilot"
MODELS="$BASE/sim_models"
VCFG="$BASE/vins_ws/src/VINS-Fusion/config/gazebo_stereo_imu_config.yaml"
HVR=/home/min/dsh095_hover.py
COLD=/home/min/dsh080e_coldstart_vision.sh
EMPTY="$PX4/Tools/sitl_gazebo/worlds/empty.world"
NW="$MODELS/vins_world"
NWORLD="$NW/vins_features.world"

echo; echo "########## [A] VINS extrinsic body_T_cam0/1 ##########"
cp -p "$VCFG" "$VCFG.pre_dsh106.bak"
echo "  before/backup md5:"; md5sum "$VCFG" "$VCFG.pre_dsh106.bak" | sed 's/^/    /'
python3 - "$VCFG" <<'PY'
import re, sys
p = sys.argv[1]
b = open(p).read()
NEW = {
 'cam0': '\n          0.0, 0.0, 1.0, 0.2,\n          -1.0, 0.0, 0.0, 0.035,\n          0.0, -1.0, 0.0, 0.035,\n          0.0, 0.0, 0.0, 1.0',
 'cam1': '\n          0.0, 0.0, 1.0, 0.2,\n          -1.0, 0.0, 0.0, -0.035,\n          0.0, -1.0, 0.0, 0.035,\n          0.0, 0.0, 0.0, 1.0',
}
out = b
ok = True
for k, v in NEW.items():
    pat = re.compile(r'(body_T_%s:\s*!!opencv-matrix\s*\n\s*rows:\s*4\s*\n\s*cols:\s*4\s*\n\s*dt:\s*d\s*\n\s*data:\s*\[)(.*?)(\])' % k, re.S)
    m = pat.search(out)
    print('  body_T_%s found=%s' % (k, bool(m)))
    if m:
        print('    old: %s' % m.group(2).strip().replace('\n', ' '))
        out = out[:m.start(2)] + v + out[m.end(2):]
    else:
        ok = False
open(p, 'w').write(out)
sys.exit(0 if ok else 3)
PY
echo "  [A] python rc=$?"
echo "  --- diff (expect exactly 2 changed data blocks) ---"
diff "$VCFG.pre_dsh106.bak" "$VCFG"
echo "  new md5: $(md5sum "$VCFG" | awk '{print $1}')"

echo; echo "########## [B] flight script dsh095_hover.py ##########"
cp -p "$HVR" "$HVR.pre_dsh106.bak"
md5sum "$HVR" | sed 's/^/  /'
echo "  --- lines before ---"
grep -n 'TAKEOFF_DZ\|LIM_Z_UP\|LIM_EV\|sz = 80.0\|dxyz > 1.2' "$HVR" | sed 's/^/    /'
python3 - "$HVR" <<'PY'
import re, sys
p = sys.argv[1]
out = open(p).read()
subs = [
 (r'(TAKEOFF_DZ\s*=\s*)[0-9.]+',  r'\g<1>1.10',  'TAKEOFF_DZ -> 1.10  (任务书 1~1.5 m)'),
 (r'(LIM_Z_UP\s*=\s*)[0-9.]+',    r'\g<1>2.20',  'LIM_Z_UP  -> 2.20'),
 (r'(LIM_EV\s*=\s*)[0-9.]+',      r'\g<1>2.20',  'LIM_EV    -> 2.20'),
 (r"(if not abort and d\['p'\]\[2\] - home\[2\] < )0\.30", r'\g<1>TAKEOFF_DZ', '起飞复查门槛 0.30 -> TAKEOFF_DZ'),
 (r'(\n\s*)sz = 80\.0',           r'\g<1>sz = 650.0', 'LOITER 防沉摇杆 80 -> 650 (0~1000, 500=中位)'),
 (r'if dxyz > 1\.2 or v3d > 1\.2:', "if (d['p'][2] - home[2]) > 2.2 or v3d > 1.2:", 'LAND 失控判据 -> 相对高度 2.2 m'),
]
bad = 0
for pat, rep, desc in subs:
    out, n = re.subn(pat, rep, out, count=1)
    print('  %-52s n=%d' % (desc, n))
    if n != 1:
        bad += 1
if bad:
    print('  ### %d substitution(s) failed - file NOT written' % bad)
    sys.exit(3)
open(p, 'w').write(out)
print('  all substitutions applied')
PY
echo "  [B] python rc=$?"
echo "  --- diff ---"
diff "$HVR.pre_dsh106.bak" "$HVR"
python3 -c "import ast;ast.parse(open('$HVR').read());print('  py syntax OK')"
echo "  new md5: $(md5sum "$HVR" | awk '{print $1}')"

echo; echo "########## [C] build feature world ##########"
mkdir -p "$NW"
echo "  empty.world md5 BEFORE: $(md5sum "$EMPTY" | awk '{print $1}')  ($(stat -c %s "$EMPTY") B)"
python3 - "$EMPTY" "$NWORLD" <<'PY'
import math, sys, xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
txt = open(src).read()
assert txt.count('</world>') == 1, 'expect exactly one </world>'
CX, CY = 1.0, 1.0
LIGHT = '0.93 0.93 0.93 1'; DARK = '0.04 0.04 0.04 1'
parts = []
def mat(rgba):
    return ("        <material>\n"
            "          <ambient>%s</ambient>\n"
            "          <diffuse>%s</diffuse>\n"
            "          <specular>0.15 0.15 0.15 1</specular>\n"
            "        </material>\n" % (rgba, rgba))
def box(name, x, y, z, sx, sy, sz, rgba):
    parts.append(
      "    <model name='%s'>\n      <static>true</static>\n"
      "      <pose>%.3f %.3f %.3f 0 0 0</pose>\n      <link name='link'>\n"
      "        <visual name='v'>\n          <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>\n"
      "%s        </visual>\n      </link>\n    </model>\n"
      % (name, x, y, z, sx, sy, sz, mat(rgba)))
def cyl(name, x, y, z, r, l, rgba):
    parts.append(
      "    <model name='%s'>\n      <static>true</static>\n"
      "      <pose>%.3f %.3f %.3f 0 0 0</pose>\n      <link name='link'>\n"
      "        <visual name='v'>\n          <geometry><cylinder><radius>%.3f</radius><length>%.3f</length></cylinder></geometry>\n"
      "%s        </visual>\n      </link>\n    </model>\n"
      % (name, x, y, z, r, l, mat(rgba)))

# 1) 两块 4x4 黑白棋盘墙（正前 / 正后），面板 0.4 m，覆盖 z 0.2~1.8 m
for tag, wx in (('A', CX + 3.2), ('B', CX - 3.2)):
    for r in range(4):
        for c in range(4):
            col = DARK if (r + c) % 2 == 0 else LIGHT
            box('wall%s_%d%d' % (tag, r, c), wx, CY - 0.6 + 0.4 * c, 0.4 + 0.4 * r,
                0.06, 0.4, 0.4, col)

# 2) 内环 r=2.6 m，12 根双色柱 h=1.5 m，方位 15+30k 度
for k in range(12):
    a = math.radians(15 + 30 * k)
    x, y = CX + 2.6 * math.cos(a), CY + 2.6 * math.sin(a)
    lo, hi = (DARK, LIGHT) if k % 2 == 0 else (LIGHT, DARK)
    box('r1_%d_lo' % k, x, y, 0.375, 0.28, 0.28, 0.75, lo)
    box('r1_%d_hi' % k, x, y, 1.125, 0.28, 0.28, 0.75, hi)

# 3) 中环 r=4.6 m，12 根双色柱 h=2.0 m，方位 30k 度
for k in range(12):
    a = math.radians(30 * k)
    x, y = CX + 4.6 * math.cos(a), CY + 4.6 * math.sin(a)
    lo, hi = (LIGHT, DARK) if k % 2 == 0 else (DARK, LIGHT)
    box('r2_%d_lo' % k, x, y, 0.500, 0.45, 0.45, 1.00, lo)
    box('r2_%d_hi' % k, x, y, 1.500, 0.45, 0.45, 1.00, hi)

# 4) 外环 r=7.0 m，8 个单色块 h=2.2 m，方位 45k+22.5 度
for k in range(8):
    a = math.radians(45 * k + 22.5)
    x, y = CX + 7.0 * math.cos(a), CY + 7.0 * math.sin(a)
    box('r3_%d' % k, x, y, 1.10, 0.80, 0.80, 2.20, DARK if k % 2 == 0 else LIGHT)

# 5) 6 根细立杆 r=3.4 m，h=2.4 m
for k in range(6):
    a = math.radians(60 * k + 30)
    x, y = CX + 3.4 * math.cos(a), CY + 3.4 * math.sin(a)
    cyl('pole_%d' % k, x, y, 1.20, 0.07, 2.40, DARK)

ins = ''.join(parts)
out = txt.replace('</world>', ins + '  </world>', 1)
open(dst, 'w').write(out)
ET.parse(dst)                      # 良构性校验
print('  models written: %d' % out.count('<model name='))
print('  file bytes    : %d' % len(out))
print('  XML parse     : OK')
PY
echo "  [C] python rc=$?"
echo "  empty.world md5 AFTER : $(md5sum "$EMPTY" | awk '{print $1}')"
ls -l "$NWORLD"

echo; echo "########## [D] cold-start script -> new world ##########"
cp -p "$COLD" "$COLD.pre_dsh106.bak"
grep -n '^WORLD=' "$COLD" | sed 's/^/  /'
python3 - "$COLD" "$NWORLD" <<'PY'
import re, sys
p, w = sys.argv[1], sys.argv[2]
b = open(p).read()
out, n = re.subn(r'(?m)^WORLD=.*$', 'WORLD="%s"' % w, b, count=1)
print('  WORLD line replaced n=%d' % n)
if n != 1:
    sys.exit(3)
open(p, 'w').write(out)
PY
echo "  [D] python rc=$?"
diff "$COLD.pre_dsh106.bak" "$COLD"
bash -n "$COLD" && echo "  cold-start bash -n OK"

echo; echo "########## [E] camera calib files ##########"
VCDIR="$(dirname "$VCFG")"
for f in gazebo_cam0_pinhole.yaml gazebo_cam1_pinhole.yaml; do
  echo "--- $f ---"
  ls -l "$VCDIR/$f" 2>/dev/null || echo "  MISSING"
  cat "$VCDIR/$f" 2>/dev/null | sed 's/^/    /'
done

echo; echo "########## [F] summary ##########"
for f in "$VCFG" "$HVR" "$COLD" "$NWORLD" "$VCFG.pre_dsh106.bak" "$HVR.pre_dsh106.bak" "$COLD.pre_dsh106.bak"; do
  echo "  $(md5sum "$f" | awk '{print $1}')  $(stat -c %s "$f") B  $f"
done
echo "==== DSH-106 done $(date -u '+%T') ===="
