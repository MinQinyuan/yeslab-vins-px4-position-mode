#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_to_px4.py —— VINS-Fusion 位姿 → MAVROS External Vision 桥接节点

项目：YESLab-Gazebo-VINS-PX4
作用：把 VINS-Fusion 的 /vins_estimator/odometry 转成 PX4 EKF2 可用的外部视觉位姿，
      发布到 MAVROS 的 /mavros/vision_pose/pose（ENU；MAVROS 内部再转 NED 送飞控）。

设计要点（对应 DSH-074 第四节 TF 风险）：
  R1  VINS odometry 实测 frame_id=child_frame_id="world" → 本节点**显式重写** header
  R2  外参：VINS 输出为 IMU 体系位姿；IMU 链接与机体系轴对齐 → 默认不额外补外参，
      预留外参参数供需要时补齐（全部变换留在本脚本内，**不改 PX4 参数**）
  R3  原点/偏航对齐：~origin_x/y/z + ~yaw_offset_deg，把 VINS 世界原点对齐到 PX4 本地原点
  R4  协方差：VINS 默认协方差为空/0，本节点写入合理对角值
  R5  时间戳守卫：丢弃倒退帧与过旧帧（复用 DSH-072 的判定思路）
  R6  可选 EV-B 通道：同时以 nav_msgs/Odometry 发到 /mavros/odometry/in 做交叉验证
  R7  不发布任何 Gazebo Ground Truth 数据（任务书十二节第 6 条严禁）

参数：
  ~input           默认 /vins_estimator/odometry
  ~output          默认 /mavros/vision_pose/pose
  ~frame_id        默认 map
  ~child_frame_id  默认 base_link
  ~origin_x/y/z    默认 0（VINS 世界 → PX4 本地的平移偏移）
  ~yaw_offset_deg  默认 0（偏航对齐角，度）
  ~pos_cov         默认 0.01
  ~rot_cov         默认 0.01
  ~max_age         默认 0.5（秒；超过则丢弃，防陈旧数据进 EKF）
  ~publish_odom_in 默认 false（true 时同时发 EV-B 通道）
  ~print_every     默认 1.0（秒；打印一次状态，便于截图取证）
"""

import math
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VinsToPx4(object):
    def __init__(self):
        self.in_topic = rospy.get_param('~input', '/vins_estimator/odometry')
        self.out_topic = rospy.get_param('~output', '/mavros/vision_pose/pose')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')

        self.ox = rospy.get_param('~origin_x', 0.0)
        self.oy = rospy.get_param('~origin_y', 0.0)
        self.oz = rospy.get_param('~origin_z', 0.0)
        self.yaw_offset = math.radians(rospy.get_param('~yaw_offset_deg', 0.0))

        self.pos_cov = rospy.get_param('~pos_cov', 0.01)
        self.rot_cov = rospy.get_param('~rot_cov', 0.01)
        self.max_age = rospy.get_param('~max_age', 0.5)
        self.publish_odom_in = rospy.get_param('~publish_odom_in', False)
        self.print_every = rospy.get_param('~print_every', 1.0)

        # 统计量（供取证截图）
        self.n_in = 0
        self.n_out = 0
        self.n_drop_back = 0
        self.n_drop_stale = 0
        self.last_stamp = None
        self.last_print = None

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=10)
        self.pub_odom = None
        if self.publish_odom_in:
            self.pub_odom = rospy.Publisher('/mavros/odometry/in', Odometry, queue_size=10)

        self.sub = rospy.Subscriber(self.in_topic, Odometry, self.cb, queue_size=50)

        rospy.loginfo("[vins_to_px4] %s -> %s | frame=%s child=%s | origin=(%.3f,%.3f,%.3f) yaw_off=%.1fdeg | max_age=%.2fs",
                      self.in_topic, self.out_topic, self.frame_id, self.child_frame_id,
                      self.ox, self.oy, self.oz, math.degrees(self.yaw_offset), self.max_age)

    # ---------- 转换 ----------
    def _apply_origin_and_yaw(self, p, q):
        """把 VINS 世界下的位姿变换到对齐后的本地系（绕 z 偏航 + 平移偏移）"""
        cy, sy = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        x = cy * p.x - sy * p.y + self.ox
        y = sy * p.x + cy * p.y + self.oy
        z = p.z + self.oz
        q_yaw = tft.quaternion_about_axis(self.yaw_offset, (0, 0, 1))
        q_new = tft.quaternion_multiply(q_yaw, (q.x, q.y, q.z, q.w))
        return x, y, z, q_new

    # ---------- 回调 ----------
    def cb(self, msg):
        self.n_in += 1
        t = msg.header.stamp

        # R5-① 时间倒退守卫（DSH-072 已证实 VINS/相机存在首帧与重复帧风险）
        if self.last_stamp is not None and t <= self.last_stamp:
            self.n_drop_back += 1
            return
        # R5-② 过旧帧守卫
        now = rospy.Time.now()
        if now.to_sec() > 0 and (now - t).to_sec() > self.max_age:
            self.n_drop_stale += 1
            return
        self.last_stamp = t

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y, z, qn = self._apply_origin_and_yaw(p, q)

        out = PoseStamped()
        out.header.stamp = t                      # 保持仿真时间域
        out.header.frame_id = self.frame_id       # R1 显式重写，不沿用 "world"
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.position.z = z
        out.pose.orientation.x = qn[0]
        out.pose.orientation.y = qn[1]
        out.pose.orientation.z = qn[2]
        out.pose.orientation.w = qn[3]
        # 注意：geometry_msgs/PoseStamped 的 Pose **没有 covariance 字段**，
        #       MAVROS vision_pose 插件也只接受 PoseStamped。
        #       ⇒ 协方差只能走 EV-B 的 nav_msgs/Odometry 通道（见 R6），
        #         或在 PX4 侧用 EKF2_EV_NOISE_MD 设定（属改参数，需批准）。

        self.pub.publish(out)
        self.n_out += 1

        # R6 可选 EV-B 对照通道（nav_msgs/Odometry 带协方差）
        if self.pub_odom is not None:
            od = Odometry()
            od.header = out.header
            od.child_frame_id = self.child_frame_id
            od.pose.pose = out.pose
            # R4 协方差（位置 0..2，姿态 3..5）
            for i in range(6):
                od.pose.covariance[i * 6 + i] = self.pos_cov if i < 3 else self.rot_cov
            od.twist.twist = msg.twist.twist
            self.pub_odom.publish(od)

        # 周期打印（截图取证用）
        if self.last_print is None or (t - self.last_print).to_sec() >= self.print_every:
            self.last_print = t
            rospy.loginfo("[vins_to_px4] n_in=%d n_out=%d drop_back=%d drop_stale=%d | pos=(%.3f, %.3f, %.3f) q=(%.4f, %.4f, %.4f, %.4f)",
                          self.n_in, self.n_out, self.n_drop_back, self.n_drop_stale,
                          x, y, z, qn[0], qn[1], qn[2], qn[3])


if __name__ == '__main__':
    rospy.init_node('vins_to_px4')
    VinsToPx4()
    rospy.spin()
