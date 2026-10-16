# 脚本/

本目录为本人自编脚本。

## 一、核心脚本（提交重点）

| 文件 | 说明 |
| --- | --- |
| `vins_to_px4.py` | **VINS 位姿 → MAVROS 外部视觉桥接节点**。位姿透传、`header` 显式重写、原点/偏航对齐、协方差注入、时间戳倒退与过旧帧守卫；**明确不订阅、不发布任何 Gazebo 真值数据**（任务书十二节第 6 条） |
| `dsh095_hover.py` | **自动飞行脚本**。起飞 → `AUTO.LOITER` 悬停 30 s → `POSCTL` 保持 30 s → `AUTO.LAND`；含安全守卫（位置/速度/EV 包线）与逐帧 CSV 记录，紧急时 `FORCE disarm(21196)` |
| `dsh080e_coldstart_vision.sh` | **整链冷启动核心**。清理旧链路（保留 rosmaster）→ 以 `vins_features.world` 启动 `gzserver` → 重新生成 `iris_vins` → 用 `rcS_vision_startup.sh` 注入副本启动 PX4 → 打印 EKF2 参数 → 启动 `vins_node` 与桥接节点 |
| `dsh106_apply_changes.sh` | 外参 / 特征世界 / 飞行脚本三处补丁的**应用与校验脚本**（含逐项 md5 核对与 diff 输出） |
| `vins_restart_and_gate.sh` | **飞行前健康门**。重启 `vins_node` → 等 15 s → 速率检查 → 内联判据（EKF 5 s 位移 < 0.05 m、EV < 0.05 m、位置 < 0.5 m、速度 < 0.1 m/s）→ 通过则打印 `GATE OK` |
| `rcS_vision_startup.sh` | PX4 启动脚本注入副本（指定 `PX4_SIM_MODEL`、启动仿真与视觉链路相关模块） |
| `restore_chain_coldstart.sh` | 链路恢复辅助脚本 |

## 二、调试 / 取证脚本

| 文件 | 说明 |
| --- | --- |
| `coordinate_transform.py` | ROS(ENU/FLU) ↔ PX4(NED/FRD) 坐标变换验证 |
| `dsh082_offb.py` / `dsh082_flight3.py` / `dsh082_flightulog.py` | 早期 Offboard 模式飞行试验 |
| `dsh082_ulog.py` / `dsh082_plot.py` | 飞控 ulog 记录读取与绘图 |
| `dsh083_readout.sh` / `dsh083_stageE.sh` | Stage-E 阶段的参数读值脚本 |
| `dsh083_analyze.py` / `dsh083_why2.py` / `dsh083_stepE2.py` / `dsh083_stepE3.py` | Stage-E 阶段的归因分析脚本 |

## 三、一个真实的坑（脚本编码）

`vins_restart_and_gate.sh` 的早期副本曾因 **UTF-8 BOM 头 + CRLF 行尾**在客机上无法执行
（`bash` 报 `$'\r': command not found`）。归一化为「去 BOM + CRLF→LF」后与正确版本逐字节一致。
坏版本备份保留在同目录 `vins_restart_and_gate.sh.BOM_CRLF.bak`，未删除。
