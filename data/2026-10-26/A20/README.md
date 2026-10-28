> **状态：📦 历史快照** ｜ 最后有效：2026-10-26（软件检查通过；v2实物/地面测试待复核）。当前赛车程序是 [A22 v2.14](../../../tools/chassis/track_race_v2/README.md)。

# A20 v2 验证记录

[当前程序与操作说明](../../../tools/chassis/ground_sequence/README.md)。本版快速转舵25µs步进，START/START1每段1秒，START3每段3秒；直行、左弧、右弧三段，前回中1秒/倒计时3秒，正常结束回中保持1.5秒。中位1375、左975、右1775µs，后轮50/255。无ToF避障。

ESP32 core3.3.12编译288491字节、静态RAM23144，上传四段Flash哈希通过。[编译/上传摘要](compile-upload-summary.json)保留源码SHA256，排除构建目录、镜像、本机路径和设备MAC。COM号以实际枚举为准。

以下均在用户确认电池拔下、D057断开绝缘、主控仅USB供电时完成，是软件指令/输出时序检查，无执行器动力：

| 检查 | 结果与日志 |
|---|---|
| 默认1秒完整顺序 | 三段下发各995ms；忙时START3拒绝，档位不变：[日志](dry-full-1s.log) |
| 显式3秒完整顺序 | 三段下发各2995ms；忙时START1拒绝：[日志](dry-full-3s.log) |
| 倒计时中止 | 锁停、不继续驱动：[日志](dry-abort-countdown.log) |
| 直行中止 | 制动并锁停：[日志](dry-abort-drive.log) |
| 静止转舵中止 | 不继续后续驱动：[日志](dry-abort-steer.log) |
| 最终复位 | IDLE、电机原始输出全0/舵机PWM关：[日志](startup-final-idle.log) |

两档预回中1000/1011ms；左转舵含500ms稳定1080ms、跨侧1720ms、结束回中阶段2080ms（含1.5秒保持）。这些是软件时序，不是电机运动时间、真实刹停或轮角反馈。未做任务阻塞故障注入。

[汇总](software-checks.json)、[验证记录](verification.json)、[用户观察与待测](observations.json)。离线复核可运行 `python -B data/2026-10-26/A20/analyze_checks.py`，仅读取日志并重算software-checks.json，不访问串口；时间戳会更新。

用户反馈上一版v1架空两轮前进/左右转向正常；v2转向更快/运行更久，仍需先架空START，再断动力放地，初次落地START。D057与电脑USB互斥，原公共地沿用。异常切执行器动力，0/!/x仅是软件请求。带载行程/速度、制动、双轮协调、实际轮角/回中和赛道运行仍未完成。

AI披露：Codex编写/修改固件、指导操作、编译烧录、分析断动力日志；物理接线、观察及测量由组员完成。复核方：组员/DSH。
