> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 编译、烧录与人工测试

## 主程序

Arduino IDE直接打开[track_racer.ino](../tools/chassis/track_racer/track_racer.ino)，保持同目录config.h/controller.h。选择ESP32 Dev Module（esp32:esp32:esp32），core3.3.12，安装Adafruit VL53L0X1.2.5和BusIO1.17.4。舵机使用core LEDC，不另装ESP32Servo。

改线/烧录时断开电池和D057，主控仅USB供电。选择本次设备实际COM号，115200；仓库证据中COM4是当时设备号，不是强制固定号。串口监视器占用端口时先关闭再上传。中文构建路径遇到链接异常可使用ASCII临时构建路径，构建输出不要放进仓库。

当前已上板参数应打印：

```text
TOF_MAP,front=4,left=13,right=17
DRIVE_SIGNS,motor=1/-1,encoder=1/-1,direction_checked=1,commissioned=0
WHEEL_CAL,cpr4=824.2/825.7,diameter_mm=68.97/68.97
```

## 串口命令

行尾选择换行。上电IDLE不驱动；R/C行持续输出，滚动不等于程序跑车。

| 命令 | 行为与前提 |
|---|---|
| BENCH | IDLE下允许人工短点动，本身不转轮 |
| JOG L / JOG R | BENCH启用后对应轮50/255运行约300ms后制动，必须四轮离地/车架固定 |
| CENTER / LEFT / RIGHT | BENCH启用后中位及±100µs小幅舵机测试，先核供电与机构余量 |
| CAL ROLL | 双轮50/255约700ms后制动；已核方向/CPR/轮径及中位后，在独立封闭直道受控人工测制动使用，不能依赖未标定传感器避墙 |
| ARM / RUN | 受完整标定和传感器门禁约束；当前不满足，自主不能启动 |
| ! / x / 0 | 软件锁停；恢复需排查后EN。IDE需点发送，异常仍优先物理切驱动动力 |

左右按站在车尾面向车头定义。轮胎底部向车尾（顶部向车头）表示推车前进。点动后停稳至少3秒再发下一条。不要同时交换电机线与软件符号。

当前原始ticks左前进增加、右前进减少；ENCODER_SIGN只修正行程/速度，不改原始ticks。CPR与轮径是初值，真实速度和停止距离仍待交叉测量。

## 日志与电脑测试

```powershell
python tools/chassis/track_racer/analyze_log.py data/2026-10-23/startup-wheelcal.log
powershell -ExecutionPolicy Bypass -File tools/chassis/track_racer/tests/run_tests.ps1
```

电脑C++测试需要MSVC Build Tools与Windows SDK；输出默认去系统临时目录。tests/fixture_config.h是虚构参数，只用于电脑控制策略测试，**不能复制成实车config.h**。历史自动上电测试程序在独立目录，默认使用track_racer。
