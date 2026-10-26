> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 当前赛车主程序

打开同目录track_racer.ino，保持config.h/controller.h；环境、命令、供电与测试流程见[固件说明](../../../docs/FIRMWARE.md)。

当前前/左/右XSH=4/13/17，电机方向+1/-1、编码器+1/-1、CPR4=824.2/825.7，两轮有效直径约68.97mm。数据来源和误差边界见[标定记录](../../../docs/CALIBRATION.md)。自主COMMISSIONED=false，未知项不猜填。

analyze_log.py离线分析R/C串口文本。tests包含电脑控制策略与默认门禁测试，fixture_config.h全部为虚构参数，不能上车。MSVC运行脚本将生成物放系统临时目录。

本目录三个生产文件与[data中的烧录验证](../../../data/2026-10-23/flash-verification.json)源码哈希一致。历史失败和完整本地工作记录未整体公开，当前上传/启动/采样证据可单独读取。
