# ENGR1000J-Group1

DeepSeeker · ENGR1000J机器人赛车项目。

当前交付更新至 **2026-10-26**。最近烧录的是 [A20 v2手动顺序试车程序](tools/chassis/ground_sequence/README.md)：直行→左弧→右弧，快速转舵，1秒/3秒档，正常运行前后回中。上电待机，必须显式START。**尚未完成地面/自主赛道验收，无自动避障。**

| 内容 | 入口 |
|---|---|
| 当前试车程序、接线和操作 | [ground_sequence](tools/chassis/ground_sequence/README.md) |
| 断动力软件检查与待测数据 | [A20验证记录](data/2026-10-26/A20/README.md) |
| 前轮五档单测工具（需另行烧录） | [car_steering v5](tools/servo/car_steering/README.md) |
| 已有证据与待测状态 | [STATUS](docs/STATUS.md) |
| 编译/烧录及各固件命令边界 | [FIRMWARE](docs/FIRMWARE.md) |
| 当前实车接线 | [WIRING](docs/WIRING.md) |
| 编码器/轮径初值与历史测距 | [CALIBRATION](docs/CALIBRATION.md) |
| 历史工具 / 测量数据 | [工具](tools/README.md) / [数据](data/README.md) |
| CAD及旧接线表适用边界 | [硬件参考](hardware/README.md) |
| AI披露 / 文件哈希 | [PROVENANCE](docs/PROVENANCE.md) / [MANIFEST](MANIFEST.json) |

ESP32-WROOM-32E、Arduino-ESP32 **3.3.12**；A20无需额外ToF或舵机库。START/START1每段1秒（三段累计3秒驱动指令），START3每段3秒（累计9秒）。115200、换行。USB供主控时D057外部5V必须断开绝缘；改线/烧录先断动力电池。

此前[自主循墙赛车源码](tools/chassis/track_racer/track_racer.ino)及2026-10-23参数保留，COMMISSIONED=false；不能用其命令控制A20。电机/编码器符号+1/-1、CPR初值824.2/825.7、三圈650mm对应有效轮径约68.97mm，均待带载复核。

本次五项检查均为断执行器动力的软件验证。用户仅报告上一版v1架空动作正常；新v2实际动作、地面速度/距离、制动、真实轮角/扫掠和整圈仍待测。仓库不含构建镜像、缓存、完整本地工作目录或本机身份信息。
