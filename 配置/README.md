# 配置/

VINS-Fusion 配置文件。

| 文件 | 说明 | 状态 |
| --- | --- | --- |
| `vins_config.yaml` | VINS-Fusion 主配置（相机内外参、相机-IMU 外参、IMU 噪声、话题名、时间同步） | ✅ 已就位 |

要点（均为实测值或由仿真模型几何推导，非官方默认值）：

- 话题：`imu_topic: "/imu0"`、`image0_topic: "/stereo/left/image_raw"`、`image1_topic: "/stereo/right/image_raw"`
- 双目：`num_of_cam: 2`、`image_width: 400`、`image_height: 400`
- 相机内参（另见 `gazebo_cam0_pinhole.yaml` / `gazebo_cam1_pinhole.yaml`）：
  `fx = fy = 2.383515418007097e+02`、`cx = cy = 2.005e+02`、畸变全 0
  —— 与 `horizontal_fov = 1.3962634 rad`（80°）的理论值 `f = 200 / tan(40°) = 238.35` **精确吻合**
- 双目基线：**0.0700 m**（由右目 `P[3] = -16.68460792604968 ⇒ 16.6846 / 238.3515` 得到）
- **相机-IMU 外参**（`estimate_extrinsic: 0`，按 Gazebo 模型 SDF 几何推导）：
  - `body_T_cam0`：`R = [[0,0,1],[-1,0,0],[0,-1,0]]`，`t = (0.2, 0.035, 0.035)`
  - `body_T_cam1`：同 `R`，`t = (0.2, -0.035, 0.035)`
  - 即相机光轴 = 机体 **+X（正前方）**
- **时间同步**：`estimate_td: 0`、`td: 0.0`
- 特征跟踪：`max_cnt: 150`、`min_dist: 30`、`freq: 20`

## 两处关键修改（均经用户批准，原件留 `.bak` 备份）

| 参数 | 改前 | 改后 | 原因 |
| --- | --- | --- | --- |
| `body_T_cam0/1` | 光轴映到机体 **−Y（正右方）** | 光轴 = 机体 **+X（正前方）** | 原值属约 90° 偏航级错误，水平通道方向与实际运动相反 |
| `estimate_td` | `1`（在线估计） | `0`（信任硬件同步） | 在线估计把 `td` 推到 0.054171 s，超过 IMU/图像实际时间余量（均值 0.0162 s），VINS 持续 `wait for imu` 饥饿 |

> 详细证据见项目内报告 `00_Project_Documents/DSH-098.md`（时间同步）与 `DSH-101.md`（外参）。
