> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 单件VL53L0X检查

按step0_selfcheck、step1_scan_id、step2_xshut、step3_range的源码头部依次检查供电/总线、ID、XSH和测距。默认ESP32 SDA21/SCL22，模块VCC3V3/GND；不要在未知XSH上强驱高电平。先单只，整车三只各有独立XSH。

step3_range每档修改TRUE_MM与SAMPLE_SECONDS，必须保存逐样本CSV和起止条件。TRUE_MM=100是测试设定，不是传感器已经测到100mm。只接受API成功且RangeStatus=0的有效读数；首/尾失联与有效间隔都要统计，8191等无效值不能当墙距。

uno_test是历史UNO单只代测，不是ESP32整车程序；UNO5V侧需合适的双向I²C电平转换，不能把5V信号直接接3.3V器件。以源码头部的具体接线及独立供电条件为准。

本车XSH前4/左13/右17，相关证据及待测边界见[当前接线](../../docs/WIRING.md)与[标定](../../docs/CALIBRATION.md)。
