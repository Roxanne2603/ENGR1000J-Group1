> **2026-10-26当前入口：** 最近Flash为[A20 v2](../tools/chassis/ground_sequence/README.md)。[本次验证](../data/2026-10-26/A20/README.md)。下文赛车Flash及器件工具说明保留历史，不能据此混用串口命令。

> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 数据与验证证据

当前批次在2026-10-23目录：

- [calibration.json](2026-10-23/calibration.json)：原始起止计数、初值、重复差和用户报告的滚动距离。
- [encoder-measurements.csv](2026-10-23/encoder-measurements.csv)：可计算的手工圈数记录，配套截图在evidence目录；截图本身不能证明实际手转方向/圈数或尺量长度，需结合用户报告。
- [startup-wheelcal.log](2026-10-23/startup-wheelcal.log)与[startup-summary.json](2026-10-23/startup-summary.json)：最新固件静态启动。最新窗口三路无有效读数，未确认目标几何；无效读数不是自由空间。
- [range-valid-window.log](2026-10-23/range-valid-window.log)与[统计](2026-10-23/range-valid-summary.json)：同日曾出现三路全有效的12秒窗口，真值未知，不能作测距精度标定。
- [flash-verification.json](2026-10-23/flash-verification.json)、[写入日志](2026-10-23/upload-verification.log)、[编译摘要](2026-10-23/compile-summary.txt)：源码哈希与Flash写入校验，小型验证文本不是构建产物。二进制仅记录哈希，未上传。
- [历史测试摘要](../docs/CALIBRATION.md)：旧单只ToF汇总和舵机观察，范围与本车三路数据不同。
- [本次发布检查](software-tests/publication-checks.json)：两个公开原始日志的分析计数复核通过；C++策略测试编译后被本机Windows应用控制阻止运行，没有将其记为通过。software-tests内2026-10-22的通过日志仅作历史证据。

时间是采集时钟，不把程序命令时长当作物理运动时长。累计worst_gap可能包含本次窗口之前的失联，不能直接当成本次采集窗口的间隔。UART会跳过日志行，日志有效率不能当全部测量尝试有效率。
