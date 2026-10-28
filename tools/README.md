> **2026-10-28 当前入口：** 最近Flash为[A22 track_race_v2 v2.14](chassis/track_race_v2/README.md)。[本次验证](../data/2026-10-28/A22/README.md)。A20 v2/[track_racer](chassis/track_racer/README.md) 为历史/单独工具，串口命令不能与 A22 混用。

> 状态：⚠️ 部分过期｜2026-10-28 更新当前入口；下文 2026-10-23 表保留历史

# 程序入口

| 路径 | 用途 | 状态 |
|---|---|---|
| [chassis/track_race_v2](chassis/track_race_v2/README.md) | **当前**连续赛道试跑（三路ToF+编码器+舵机） | v2.14已烧录；首个圆岛右弯未通过 |
| [chassis/ground_sequence](chassis/ground_sequence/README.md) | A20 v2手动直行/左弧/右弧三段 | 单独烧录工具，非自主赛车 |
| [chassis/track_racer](chassis/track_racer/README.md) | 2026-10-23整合赛车程序（历史） | 上电IDLE，自主标定未完成 |
| [md520](md520/README.md) | 单件编码器初筛/ISR压力检查 | 按对应源码头部与丝印接线 |
| [vl53l0x](vl53l0x/README.md) | 单件寻址/XSH/测距，含UNO临时代测 | 单件统计不当作三路装车标定 |
| [servo](servo/README.md) | 舵机初测/带前轮/分档扩幅 | 先核独立供电和机械余量 |
| [chassis/power_on_test](chassis/power_on_test/README.md) | 历史A15一次性自动动作 | 上电自动动作，不能作为当前默认程序 |

这些目录是不同测试场景，不应同时烧录或混用接线与标定。实际开电、烧录及动作由操作人按[供电说明](../docs/WIRING.md)逐项确认。
