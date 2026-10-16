#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
coordinate_transform.py —— ROS(ENU/FLU) ↔ PX4(NED/FRD) 坐标变换工具

项目：YESLab-Gazebo-VINS-PX4
用途：
  ① 为 vins_to_px4.py 提供可复用的坐标系/外参工具
  ② 单独运行时可做**坐标方向自检**，验证接入 EV 前的位置/姿态方向是否正确
     （任务书十二节第 6 条严禁用 Gazebo Ground Truth 作为位置输入；
       本工具只做"方向是否合理"的离线核验，不参与飞控输入）

坐标约定（ROS REP-103 / PX4 官方）：
  ROS  : ENU（x 东, y 北, z 天）  机体 FLU（x 前, y 左, z 上）
  PX4  : NED（x 北, y 东, z 地）  机体 FRD（x 前, y 右, z 下）
  关系 : NED = R * ENU，其中 R = [[0,1,0],[1,0,0],[0,0,-1]]
         MAVROS 在做 vision_pose 时**会自动**完成该 ENU→NED 转换，
         因此桥接节点只需输出 ENU，本模块用于**验证与自检**。

自检用法：
  python3 coordinate_transform.py --self-test
"""

from __future__ import print_function
import argparse
import math

try:
    import numpy as np
except ImportError:
    np = None


# ---------------- 基础旋转 ----------------
def R_enu_to_ned():
    """ENU → NED（与 MAVROS 一致的经典变换）"""
    if np is None:
        return [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    return np.array([[0.0, 1.0, 0.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 0.0, -1.0]])


def R_flu_to_frd():
    """FLU(ROS 机体系) → FRD(PX4 机体系)"""
    if np is None:
        return [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0],
                     [0.0, 0.0, -1.0]])


def enu_to_ned(x, y, z):
    """位置：ENU → NED"""
    return (y, x, -z)


def ned_to_enu(x, y, z):
    """位置：NED → ENU（自身逆变换）"""
    return (y, x, -z)


def quat_yaw(q):
    """从四元数 (x,y,z,w) 取偏航角（弧度）"""
    x, y, z, w = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def quat_about_axis(angle, axis=(0, 0, 1)):
    """绕指定轴的四元数（不依赖 tf，便于离线自检）"""
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n == 0:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = ax / n, ay / n, az / n
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


def quat_multiply(q1, q2):
    """四元数乘法 q1 * q2，(x,y,z,w)"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# ---------------- 自检 ----------------
def self_test():
    ok = True
    print("=== 坐标变换自检 ===")

    # 1) ENU 东(+x) → NED 应为 东 = +y
    r = enu_to_ned(1.0, 0.0, 0.0)
    print("  ENU 东(1,0,0)  -> NED", tuple(round(v, 3) for v in r))
    ok &= (abs(r[0]) < 1e-9 and abs(r[1] - 1.0) < 1e-9 and abs(r[2]) < 1e-9)
    print("    期望 (0,1,0) —— 东在 NED 中为 +y :", "OK" if ok else "FAIL")

    # 2) ENU 天(+z) → NED 应为 地 = -z
    r = enu_to_ned(0.0, 0.0, 1.0)
    print("  ENU 天(0,0,1)  -> NED", tuple(round(v, 3) for v in r))
    ok &= (abs(r[0]) < 1e-9 and abs(r[1]) < 1e-9 and abs(r[2] + 1.0) < 1e-9)
    print("    期望 (0,0,-1) —— 天在 NED 中为 -z :", "OK" if ok else "FAIL")

    # 3) 往返一致
    p = (1.23, -4.56, 7.89)
    back = ned_to_enu(*enu_to_ned(*p))
    same = all(abs(a - b) < 1e-9 for a, b in zip(p, back))
    print("  ENU->NED->ENU 往返:", tuple(round(v, 3) for v in back), "OK" if same else "FAIL")
    ok &= same

    # 4) 偏航四元数可被正确解析
    ang = math.radians(30.0)
    q = quat_about_axis(ang, (0, 0, 1))
    y = quat_yaw(q)
    good = abs(y - ang) < 1e-9
    print("  绕 z 转 30° 的四元数 yaw = %.2f° :" % math.degrees(y), "OK" if good else "FAIL")
    ok &= good

    # 5) 与 numpy 矩阵互验（若可用）
    if np is not None:
        v = np.array([1.0, 2.0, 3.0])
        r_np = tuple(np.dot(R_enu_to_ned(), v))
        r_my = enu_to_ned(1.0, 2.0, 3.0)
        same = all(abs(a - b) < 1e-9 for a, b in zip(r_np, r_my))
        print("  numpy 矩阵 vs 手写变换:", r_np, "OK" if same else "FAIL")
        ok &= same
    else:
        print("  (numpy 不可用，跳过矩阵互验)")

    print("=== 自检结果:", "全部通过 ✅" if ok else "存在失败 ❌", "===")
    return 0 if ok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--self-test', action='store_true', help='运行坐标方向自检')
    ap.add_argument('--yaw-offset-deg', type=float, default=0.0, help='仅演示用')
    a = ap.parse_args()
    if a.self_test:
        raise SystemExit(self_test())
    print(__doc__)
