# ENGR1000J-Group1

DeepSeeker · ENGR1000J 机器人赛车项目。

本仓库保存截至 **2026-10-23** 的核心源码、接线资料和可追溯的测试证据。当前主程序是 [track_racer.ino](tools/chassis/track_racer/track_racer.ino)，并非已经完成整车验收的比赛版本。

## 从这里开始

| 内容 | 入口 |
|---|---|
| 当前状态、已验证内容与待测项目 | [docs/STATUS.md](docs/STATUS.md) |
| 编译、烧录、命令与点动说明 | [docs/FIRMWARE.md](docs/FIRMWARE.md) |
| 当前实车接线；左右探头已交换 | [docs/WIRING.md](docs/WIRING.md) |
| 方向、每圈计数、轮径和历史测距数据 | [docs/CALIBRATION.md](docs/CALIBRATION.md) |
| 历史测试程序及适用范围 | [tools/README.md](tools/README.md) |
| 测量、启动和写入校验证据 | [data/README.md](data/README.md) |
| CAD、系统图和旧接线表的适用边界 | [hardware/README.md](hardware/README.md) |
| AI辅助披露与发布清单 | [docs/PROVENANCE.md](docs/PROVENANCE.md) |

## 当前程序状态

- ESP32-WROOM-32E；Arduino-ESP32 **3.3.12**；Adafruit VL53L0X **1.2.5**、Adafruit BusIO **1.17.4**。
- 主控 GPIO：前/左/右 XSH **4/13/17**；电机方向 **+1/-1**；编码器符号 **+1/-1**。
- 每圈四倍频计数初值 **824.2 / 825.7**；两轮三圈各滚动 **650 mm**，有效轮径约 **68.97 mm**。
- 当前源码与已上板源码哈希一致，COM4 上传四段数据校验通过；启动保持 IDLE，采集窗口内电机指令为 0。
- **COMMISSIONED=false**。车体尺寸、真实轮角/扫掠、制动能力、传感器误差和持续有效性等尚未完成，不能直接比赛或只修改这个开关就开跑。

供电切换、人工点动前请阅读接线和固件说明。USB调试时断开D057外部5V；改线/烧录时断开动力电池。电机动力线与信号杜邦线分开处理。

此仓库只包含允许清单内的源码、文档、设计源件和小型实测日志。未提交编译镜像、缓存、虚拟环境、临时截图、整份本地聊天/工作目录或无关课程作业。文件清单与SHA256见 [MANIFEST.json](MANIFEST.json)。
