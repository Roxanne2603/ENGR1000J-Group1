> **2026-10-26当前入口：** 最近Flash为[A20 v2](../tools/chassis/ground_sequence/README.md)。[本次验证](../data/2026-10-26/A20/README.md)。下文赛车Flash及器件工具说明保留历史，不能据此混用串口命令。

> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 程序入口

| 路径 | 用途 | 状态 |
|---|---|---|
| [chassis/track_racer](chassis/track_racer/README.md) | 当前整合赛车程序 | 上电IDLE，自主标定尚未完成 |
| [md520](md520/README.md) | 单件编码器初筛/ISR压力检查 | 按对应源码头部与丝印接线，不能替代本车两轮验收 |
| [vl53l0x](vl53l0x/README.md) | 单件寻址/XSH/测距，含UNO临时代测 | 单件统计不当作三路装车标定 |
| [servo](servo/README.md) | 舵机初测/带前轮/分档扩幅 | 先核独立供电和机械余量 |
| [chassis/power_on_test](chassis/power_on_test/README.md) | 历史A15一次性自动动作 | 上电自动动作，不能作为当前默认程序 |

这些目录是不同测试场景，不应同时烧录或混用接线与标定。实际开电、烧录及动作由操作人按[供电说明](../docs/WIRING.md)逐项确认。
