# 发射/

**本项目未使用 .launch 文件启动**，而是用 shell 脚本按强顺序把整条链路拉起。
详细原因与对照见同目录 [README.txt](README.txt)。

任务书 P88 建议的 `发射/*.launch`，由 `脚本/` 下的脚本等价替代：

| 任务书建议的文件 | 本项目的等价物 |
| --- | --- |
| `发射/vins_*.launch` | `脚本/dsh080e_coldstart_vision.sh`（启动 `vins_node` 与 `vins_to_px4.py` 桥接节点） |
| `发射/mavros_*.launch` | 直接使用 mavros 包自带：`roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580` |

之所以不做 launch 封装：整链启动的前两步（清理残留链路、以指定 world 启动 `gzserver`）
必须在 Gazebo / PX4 进程之外用 shell 完成，写成 launch 文件需嵌入大量 `<node>` 与 shell 混合逻辑，
实际使用反而更难维护。
