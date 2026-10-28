# ENGR1000J-Group1

DeepSeeker · ENGR1000J 机器人赛车项目。

当前交付更新至 **2026-10-28**。最近烧录的是 [A22 track_race_v2 v2.14](tools/chassis/track_race_v2/README.md)：三路 ToF + 编码器闭环的连续赛道试跑程序，自动「10 秒准备 + 1 秒回中 + 3 秒倒计时」运行一次，最长 180 秒。**首个圆岛右弯仍未稳定通过，不能当作完成整圈或可比赛版本。**

| 内容 | 入口 |
|---|---|
| 当前 A22 程序、参数和操作 | [track_race_v2](tools/chassis/track_race_v2/README.md) |
| A22 当前验证/现场记录 | [data/2026-10-28/A22](data/2026-10-28/A22/README.md) |
| 三路固定距离与量程档 | [传感器数据](data/2026-10-28/sensor-distances/README.md) / [量程档](data/2026-10-28/range-profiles/README.md) |
| 当前状态、已验证/待测 | [STATUS](docs/STATUS.md) |
| 编译、烧录、串口命令 | [FIRMWARE](docs/FIRMWARE.md) |
| 实车接线 | [WIRING](docs/WIRING.md) |
| 编码器/轮径/测距初值 | [CALIBRATION](docs/CALIBRATION.md) |
| A20 v2 手动顺序试车（另行烧录的工具） | [ground_sequence](tools/chassis/ground_sequence/README.md) |
| 前轮五档单测（另行烧录） | [car_steering v5](tools/servo/car_steering/README.md) |
| 历史赛车程序/工具/数据 | [tools](tools/README.md) / [data](data/README.md) |
| CAD/旧接线表边界 | [硬件参考](hardware/README.md) |
| AI 披露/文件哈希 | [PROVENANCE](docs/PROVENANCE.md) / [MANIFEST](MANIFEST.json) |

ESP32-WROOM-32E、Arduino-ESP32 **3.3.12**。A22 自带修改过的 Adafruit VL53L0X 1.2.5 驱动；A20 不需要 ToF/舵机额外库。**电脑 USB 与 D057 外部 5V 互斥**：烧录/看日志时拔电池、断开并绝缘 D057、仅 USB；上动力前先拔 USB、接 D057/电池。

A22 当前：349755 B 程序、27640 B 静态 RAM、COM4/115200 四段写入校验通过；仅 USB 上电路径和编译期断言通过。**首个圆岛右弯、整圈、带载速度/制动、R400/R850 扫掠、真实轮角/回中和比赛可靠性均未验收。** `COMMISSIONED=false`。

此前 [A20 v2 手动顺序](tools/chassis/ground_sequence/README.md) 与 [track_racer](tools/chassis/track_racer/README.md) 保留为历史/单独工具；串口命令不可与 A22 混用。电机/编码器符号 +1/−1、CPR 初值 824.2/825.7、三圈 650mm 对应有效轮径约 68.97mm，仍需带载复核。

本仓库只包含允许清单内的源码、文档、设计源件和可公开的小型实测日志；不含编译镜像、缓存、完整本地工作目录、设备 MAC 或本机路径。文件清单与 SHA256 见 [MANIFEST.json](MANIFEST.json)。
