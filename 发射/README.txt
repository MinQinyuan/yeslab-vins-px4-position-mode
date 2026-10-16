发射/ —— 本项目不使用 ROS launch 文件启动（说明）

任务书 P88 建议提交 发射/*.launch。本项目实际未使用 .launch 文件启动，
原因如下：

  1. 整链启动涉及「清理旧链路 → 以指定 world 启动 gzserver → 重新生成
     iris_vins 模型 → 用 rcS 注入副本启动 PX4 → 启动 VINS → 启动桥接节点」
     这一串强顺序动作，其中前两步必须在 Gazebo/PX4 之外用 shell 完成，
     写成 launch 文件反而要嵌入大量 <node> + shell 混合逻辑。
  2. MAVROS 的启动确实是一条标准 roslaunch 命令，已写在 README 第 4 节：
         roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
     它直接使用 mavros 包自带的 px4.launch，无需另行提交副本。

实际启动方式（替代 launch 文件）：

    bash /home/min/dsh085_bringup.sh          # 一次性整链冷启动（约 4.5 分钟）
    bash /home/min/vins_restart_and_gate.sh   # 飞行前健康门 → GATE OK
    bash /home/min/dsh087b_shortgate.sh       # 短复核     → SHORT GATE OK
    bash /home/min/dsh095b_run.sh             # 录 bag + 飞行

对应的脚本副本已放在 脚本/ 目录：

    脚本/dsh080e_coldstart_vision.sh   整链冷启动的核心脚本（被 bringup 调用）
    脚本/rcS_vision_startup.sh         PX4 启动脚本注入副本（启动 PX4 用）

若评审方仍希望有 .launch 形式，可基于以上脚本再封装，本项目未做该封装。
