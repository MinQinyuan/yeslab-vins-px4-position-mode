图片/ —— 截图清单

任务书 P88 / P95~P97 要求的 4 张截图（均为本人复现过程中实测截取）：

  gazebo.png               Gazebo + PX4 SITL 正常运行
                           （可见三维场景渲染、Real Time Factor 1.00、px4 starting.、
                             Simulator connected on TCP port 4560）

  vins_rviz.png            RViz 中的 VINS 连续轨迹
                           （/vins_estimator/path + camera_pose_visual + point_cloud，
                             轨迹连续延伸而非单帧孤点）

  px4_local_position.png   PX4 Local Position 有效的证据
                           （/mavros/local_position/pose 连续输出 30.303 Hz，
                             伴随 /mavros/vision_pose/pose 10 Hz）

  position_mode.png        进入 Position（POSCTL）模式并悬停的证据
                           （DSH-108：LOITER 连续 30.0 s + POSCTL 30.0 s，
                            高度 1.063~1.384 m / 1.107~1.159 m）

额外补充的证据截图（本项目自加）：

  stepE_terminal.png       Day1 终端完整记录（含子模块修复与构建过程）
  pxh_prompt.png           PX4 Shell（pxh>）进入证据
  stepF_mavros.png         MAVROS 联调证据（connected: True）

引用方式：README.md 中按相对路径引用，如 `图片/gazebo.png`。
上传到 GitHub 后应能在仓库中直接点开查看。

注：本目录不含 Gazebo Ground Truth 作为位置输入的任何证据（任务书十二节第 6 条严禁），
第 6 节引用的真值对照为事后误差比对，不是控制回路输入。
