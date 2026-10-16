
# -*- coding: utf-8 -*-
"""DSH-116/118 flight replay renderer (host side, Pillow only).

Input : flight CSV written by dsh116*_movetest.py
        t,phase,action,lp_x,lp_y,lp_z,lv_x,lv_y,lv_z,ev_x,ev_y,ev_z,
        vins_x,vins_y,vins_z,tx,ty,tz,rc_roll,rc_pitch,rc_thr,rc_yaw,mode,armed,dxy,dz,tdxy,tdz
Output: <out>.webp (quality) + <out>.gif (palette, for markdown embedding)

Panels: top-down XY truth/EKF/VINS trajectory (growing), altitude vs time,
phase timeline, live status text.  Ground truth is display-only.
"""
import argparse
import csv
import math
from PIL import Image, ImageDraw, ImageFont

W, H = 720, 440
BG = (16, 20, 26)
FG = (232, 238, 245)
DIM = (140, 152, 166)
GRID = (38, 46, 56)
C_TRUTH = (86, 220, 130)
C_EKF = (86, 178, 255)
C_VINS = (255, 170, 60)
C_HOME = (255, 90, 90)
BAND = (46, 60, 40)

PHC = {
    'PRE': (90, 96, 104), 'ARM': (70, 110, 190), 'TAKEOFF': (40, 170, 160),
    'TO_LOITER': (40, 200, 200), 'LOITER': (70, 200, 90),
    'TO_POSCTL': (170, 170, 60), 'POSCTL': (240, 150, 40),
    'B0_HOLD': (250, 220, 70),
    'T3_FWD': (200, 120, 240), 'T3_FWD_HOLD': (170, 90, 220),
    'T3_BACK': (150, 210, 250), 'T3_BACK_HOLD': (110, 170, 230),
    'T3_LEFT': (250, 140, 190), 'T3_LEFT_HOLD': (215, 100, 160),
    'T3_RIGHT': (190, 250, 140), 'T3_RIGHT_HOLD': (150, 215, 100),
    'T4_CLIMB': (255, 190, 120), 'T4_CLIMB_HOLD': (230, 160, 90),
    'T4_DESCEND': (180, 160, 255), 'T4_DESCEND_HOLD': (140, 120, 230),
    'LAND': (250, 80, 80),
}
PH_DEFAULT = (120, 130, 140)

# ---- panel boxes (x0, y0, x1, y1) ----
P_XY = (14, 52, 366, 404)
P_ALT = (382, 52, 706, 208)
P_TL = (382, 214, 706, 262)
P_TXT = (382, 270, 706, 404)


def font(sz):
    for p in (r'C:\Windows\Fonts\msyh.ttc', r'C:\Windows\Fonts\arial.ttf'):
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            pass
    return ImageFont.load_default()


F_T = font(15)
F_S = font(12)
F_N = font(11)


def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8', errors='replace') as f:
        for r in csv.DictReader(f):
            try:
                d = {}
                for k, v in r.items():
                    if k in ('phase', 'action', 'mode', 'armed'):
                        d[k] = v
                    else:
                        d[k] = float(v)
                rows.append(d)
            except (ValueError, TypeError):
                continue
    return rows


def rel(rows):
    t0 = (rows[0]['tx'], rows[0]['ty'], rows[0]['tz'])
    for r in rows:
        r['TDX'] = r['tx'] - t0[0]
        r['TDY'] = r['ty'] - t0[1]
        r['TDZ'] = r['tz'] - t0[2]
    return rows


def bounds(rows, keys, pad=0.35):
    lo = min(min(r[k] for r in rows) for k in keys)
    hi = max(max(r[k] for r in rows) for k in keys)
    if hi - lo < 1e-6:
        hi, lo = lo + 1.0, lo - 1.0
    m = (hi - lo) * pad
    return lo - m, hi + m


def frame(rows, idx, src, meta):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    d.text((14, 8), u'YESLab · VINS-Fusion 视觉定位 + PX4 Position 模式飞行复现', font=F_T, fill=FG)
    d.text((14, 30), u'数据源 %s　真值=Gazebo model_states（仅用于对比，不参与控制）　VINS headless' % src, font=F_N, fill=DIM)

    # ---------------- XY panel ----------------
    x0, y0, x1, y1 = P_XY
    d.rectangle([x0, y0, x1, y1], outline=GRID)
    xs = [r['TDX'] for r in rows] + [r['lp_x'] for r in rows] + [r['vins_x'] for r in rows]
    ys = [r['TDY'] for r in rows] + [r['lp_y'] for r in rows] + [r['vins_y'] for r in rows]

    def rq(v, q):
        s = sorted(v)
        return s[min(len(s) - 1, max(0, int(q * (len(s) - 1))))]

    cx = 0.5 * (rq(xs, 0.02) + rq(xs, 0.98))
    cy = 0.5 * (rq(ys, 0.02) + rq(ys, 0.98))
    span = max(rq(xs, 0.98) - rq(xs, 0.02), rq(ys, 0.98) - rq(ys, 0.02), 1.6) * 1.15
    sx = (x1 - x0 - 8) / span
    sy = (y1 - y0 - 8) / span
    s = min(sx, sy)
    ox = 0.5 * (x0 + x1)
    oy = 0.5 * (y0 + y1)

    def px(dx, dy):
        return ox + (dx - cx) * s, oy - (dy - cy) * s

    # 1 m grid
    g = math.ceil((cx - span / 2))
    while g <= cx + span / 2:
        a = px(g, cy - span / 2)
        b = px(g, cy + span / 2)
        d.line([a[0], y0 + 2, a[0], y1 - 2], fill=GRID)
        g += 1.0
    g = math.ceil((cy - span / 2))
    while g <= cy + span / 2:
        a = px(cx - span / 2, g)
        d.line([x0 + 2, a[1], x1 - 2, a[1]], fill=GRID)
        g += 1.0
    # scale bar 1 m
    sb = px(cx - span / 2 + 0.15, cy - span / 2 + 0.25)
    d.line([sb[0], sb[1], sb[0] + s, sb[1]], fill=DIM, width=2)
    d.text((sb[0], sb[1] - 14), u'1 m', font=F_N, fill=DIM)
    d.text((x0 + 6, y1 - 16), u'XY 俯视（绿=真值 蓝=EKF 橙=VINS）', font=F_N, fill=DIM)

    def path(series, col, wid=2):
        pts = [px(r[series][0], r[series][1]) for r in rows[:idx + 1]]
        if len(pts) > 1:
            d.line(pts, fill=col, width=wid, joint='curve')

    path('T', C_TRUTH, 3) if False else None
    tr = [(r['TDX'], r['TDY']) for r in rows[:idx + 1]]
    lp = [(r['lp_x'], r['lp_y']) for r in rows[:idx + 1]]
    vn = [(r['vins_x'], r['vins_y']) for r in rows[:idx + 1]]
    for seq, col, wid in ((tr, C_TRUTH, 3), (lp, C_EKF, 2), (vn, C_VINS, 2)):
        pts = []
        for a, b in seq:
            p = px(a, b)
            if x0 - 4 <= p[0] <= x1 + 4 and y0 - 4 <= p[1] <= y1 + 4:
                pts.append(p)
        if len(pts) > 1:
            d.line(pts, fill=col, width=wid, joint='curve')
    hx, hy = px(0.0, 0.0)
    d.line([hx - 5, hy, hx + 5, hy], fill=C_HOME, width=2)
    d.line([hx, hy - 5, hx, hy + 5], fill=C_HOME, width=2)
    for a, b, col in ((tr[-1], C_TRUTH, C_TRUTH), (lp[-1], C_EKF, C_EKF), (vn[-1], C_VINS, C_VINS)):
        p = px(a[0], a[1])
        d.ellipse([p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4], fill=col, outline=(10, 12, 16))

    # ---------------- altitude panel ----------------
    ax0, ay0, ax1, ay1 = P_ALT
    d.rectangle([ax0, ay0, ax1, ay1], outline=GRID)
    alo, ahi = bounds(rows, ['TDZ', 'lp_z', 'vins_z'], 0.12)
    alo = min(alo, -0.2)
    tmax = rows[-1]['t'] or 1.0

    def ap(t, z):
        return ax0 + 4 + (t / tmax) * (ax1 - ax0 - 8), ay1 - 6 - ((z - alo) / (ahi - alo)) * (ay1 - ay0 - 14)

    for zb in (0.0, 0.5, 1.0, 1.5, 2.0):
        if alo < zb < ahi:
            p = ap(0, zb)
            d.line([ax0 + 2, p[1], ax1 - 2, p[1]], fill=GRID)
            d.text((ax0 + 4, p[1] - 12), u'%+.1f' % zb, font=F_N, fill=DIM)
    b1 = ap(0, 1.0)[1]
    b2 = ap(0, 1.5)[1]
    d.rectangle([ax0 + 2, b2, ax1 - 2, b1], fill=BAND)
    for key, col, wid in (('TDZ', C_TRUTH, 3), ('lp_z', C_EKF, 2), ('vins_z', C_VINS, 2)):
        pts = [ap(r['t'], r[key]) for r in rows[:idx + 1]]
        if len(pts) > 1:
            d.line(pts, fill=col, width=wid, joint='curve')
    cur = rows[idx]
    d.line([ap(cur['t'], alo)[0], ay0 + 2, ap(cur['t'], alo)[0], ay1 - 2], fill=(255, 255, 255))
    d.text((ax0 + 6, ay0 + 3), u'高度 vs 时间 / m（阴影带 1.0–1.5 m 任务目标）', font=F_N, fill=DIM)

    # ---------------- phase timeline ----------------
    tx0, ty0, tx1, ty1 = P_TL
    d.rectangle([tx0, ty0, tx1, ty1], outline=GRID)
    for r in rows:
        xx = tx0 + 3 + (r['t'] / tmax) * (tx1 - tx0 - 6)
        col = PHC.get(r['phase'], PH_DEFAULT)
        d.line([xx, ty0 + 4, xx, ty1 - 4], fill=col)
    cx2 = tx0 + 3 + (cur['t'] / tmax) * (tx1 - tx0 - 6)
    d.line([cx2, ty0 + 1, cx2, ty1 - 1], fill=(255, 255, 255), width=2)
    d.text((tx0 + 4, ty1 - 13), u'相位时间轴（t=0…%.0f s）' % tmax, font=F_N, fill=DIM)

    # ---------------- status text ----------------
    gx0, gy0, gx1, gy1 = P_TXT
    d.rectangle([gx0, gy0, gx1, gy1], outline=GRID)
    armed = str(cur['armed'])
    lines = [
        (u't = %6.1f s   相位 %s' % (cur['t'], cur['phase']), FG),
        (u'模式 %s   armed %s' % (cur['mode'], armed), FG),
        (u'EKF  dz %+0.3f m   |Δxy| %0.3f m' % (cur['lp_z'], cur['dxy']), C_EKF),
        (u'真值 dz %+0.3f m   真值 水平位移 %0.3f m' % (cur['TDZ'], cur.get('tdxy', 0.0)), C_TRUTH),
        (u'VINS dz %+0.3f m' % cur['vins_z'], C_VINS),
        (u'RC roll %4.0f  pitch %4.0f  油门 %4.0f  yaw %4.0f' % (cur['rc_roll'], cur['rc_pitch'], cur['rc_thr'], cur['rc_yaw']), DIM),
        (u'速度 |v| %0.3f m/s   %s' % (math.sqrt(cur['lv_x'] ** 2 + cur['lv_y'] ** 2 + cur['lv_z'] ** 2), meta), DIM),
    ]
    yy = gy0 + 6
    for s, col in lines:
        d.text((gx0 + 6, yy), s, font=F_S, fill=col)
        yy += 19
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('-o', '--out', default='demo')
    ap.add_argument('--frames', type=int, default=240)
    ap.add_argument('--fps', type=float, default=5.0)
    ap.add_argument('--meta', default='')
    ap.add_argument('--pngdir', default='')
    a = ap.parse_args()
    rows = rel(load(a.csv_path))
    if len(rows) < 3:
        raise SystemExit('too few rows')
    imgs = []
    n = a.frames
    for i in range(n):
        idx = int(round(i * (len(rows) - 1) / (n - 1)))
        imgs.append(frame(rows, idx, a.csv_path.split('\\')[-1], a.meta))
    dur = int(round(1000.0 / a.fps))
    imgs[0].save(a.out + '.webp', save_all=True, append_images=imgs[1:], duration=dur,
                 loop=0, quality=82, method=4)
    pal = [im.convert('P', palette=Image.ADAPTIVE, colors=128) for im in imgs]
    pal[0].save(a.out + '.gif', save_all=True, append_images=pal[1:], duration=dur, loop=0, optimize=True)
    if a.pngdir:
        import os
        os.makedirs(a.pngdir, exist_ok=True)
        for i, im in enumerate(imgs):
            im.save(os.path.join(a.pngdir, 'f%05d.png' % i))
        print('wrote %d PNG frames to %s' % (n, a.pngdir))
    print('wrote %s.webp and %s.gif  frames=%d fps=%.1f duration=%.1fs' % (
        a.out, a.out, n, a.fps, n / a.fps))


if __name__ == '__main__':
    main()
