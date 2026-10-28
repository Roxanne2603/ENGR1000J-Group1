> **状态：⚠️ 部分过期** ｜ 2026-10-28 更新：当前 Flash 为 A22 v2.14；A20/track_racer 为历史或单独工具。

当前请在 Arduino IDE 打开 [track_race_v2.ino](../tools/chassis/track_race_v2/track_race_v2.ino)，保持同目录 `policy.h`、`sensors.h`、`fault_history.h`、`tests/` 和 `src/driver/`。板型 `ESP32 Dev Module`（esp32:esp32:esp32），core **3.3.12**；本目录自带修改过的 Adafruit VL53L0X 1.2.5 驱动，不依赖全局 Adafruit 库。完整参数/行为见[当前说明](../tools/chassis/track_race_v2/README.md)，证据见[2026-10-28 A22](../data/2026-10-28/A22/README.md)。

## A22 v2.14 编译/烧录

- 最近验证：**349755 B 程序 / 27640 B 静态 RAM**；COM4、115200 上传，四段 `Hash of data verified`；app SHA256 `3C742894E4B3EA3F258319B86C15C9A58C2C55EB97D5605E69BE8438013EA747`。
- 烧录前：拔动力电池、断开并绝缘 D057、主控仅 USB；关掉占用串口的监视器。改线/上动力前先全断电。
- 上电/复位后自动走「10 秒准备 + 1 秒回中 + 3 秒倒计时」，满足准备条件后运行一次；**没有串口 RUN 绕过**。串口 115200 + 换行：
  - `STATUS`：阶段、原因、duty、舵机、原始值、lease/transport、`open_start`、复位原因；
  - `HISTORY`：NVS 最近 8 条故障，含 `F/L/R/valid/age/servo/turn/flags`；非活动阶段才打印；
  - `!` / `0` / `x`：软件锁停；不是切电急停。
- `tests/control_checks.cpp` 是编译期断言，随编译执行；`tests/transport_compile_checks.cpp` 对实际 I2C 传输模板做编译期错误注入断言，不是实板故障注入。
- A22 不识别红点终点/计圈；180 秒自动制动，人工切驱动/总电源结束。

## A20 v2 手动顺序（单独工具）

[ground_sequence.ino](../tools/chassis/ground_sequence/ground_sequence.ino)，同目录 `sequence.h`；core3.3.12，无额外库。上电待机，必须显式 `START`/`START1`（每段 1 秒）或 `START3`（每段 3 秒）；`STATUS` 查询，`0/x/!` 中止锁停。中位 1375、左 975、右 1775µs，后轮 50/255。**A20 命令与 A22 不通用。**

## 2026-10-23 track_racer 历史说明

> 历史快照｜未完成整车标定

Arduino IDE 直接打开[track_racer.ino](../tools/chassis/track_racer/track_racer.ino)，保持同目录 `config.h`/`controller.h`；Adafruit VL53L0X 1.2.5 + BusIO 1.17.4。上电 IDLE，不自动驱动。历史命令 `BENCH`、`JOG L/R`、`CENTER/LEFT/RIGHT`、`CAL ROLL`、`ARM/RUN` 只属于该程序；`ARM/RUN` 受完整标定/传感器门禁约束，当时不满足。`! / x / 0` 软件锁停；异常仍优先物理切驱动动力。

左右按站在车尾面向车头定义；轮胎底部向车尾表示推车前进。点动后停稳至少 3 秒再发下一条。原始 ticks 左前进增加、右前进减少；`ENCODER_SIGN` 只修正行程/速度。CPR/轮径是初值，真实速度和停止距离仍待测。电脑测试：

```powershell
python tools/chassis/track_racer/analyze_log.py data/2026-10-23/startup-wheelcal.log
powershell -ExecutionPolicy Bypass -File tools/chassis/track_racer/tests/run_tests.ps1
```

`tests/fixture_config.h` 是虚构参数，只能用于电脑策略测试，不能复制成实车 `config.h`。
