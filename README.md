# CW32L010 FOC 电机控制项目

本工程实现基于 CW32L010 的 FOC（磁场定向控制）电机控制，包含位置/速度/电流环、PID、滤波与ADC采样校准等功能。

## 主要特性
- FOC 控制框架（电角度/力矩映射、PWM 输出）
- 定点 dq 电流环、速度环、位置环、力位、轨迹控制和双向棘轮
- 电流传感器支持：回调式 ADC 读取 + 零点均值校准
- 与 MT6701 位置传感器、DRV8313 驱动板集成
- CLion + CMake 工程，使用 Arm GNU 工具链和 CW32L010 SPL

## 目录

| 路径 | 内容 |
| --- | --- |
| [`docs/hardware/`](docs/hardware/README.md) | 板卡资料和从固件反推的硬件接口 |
| `firmware/` | CW32L010 固件与 CLion/CMake 工程 |
| `tools/` | PC 端构建脚本、pyOCD 配置和 [RTT CLI](tools/host/foc.py) |

## 固件结构

| 目录 | 职责 |
| --- | --- |
| `firmware/App/` | 电机参数、对齐和校准顺序、位置与速度环调度、RTT 命令与帧格式 |
| `firmware/Control/` | 定点 Clarke/Park、SVPWM、带抗饱和的 PI、七种有感控制策略和加减速轨迹。CMake 单独编译为 `motor_control` |
| `firmware/BSP/` | 时钟、计时器、ADC、DRV8313、MT6701 和板载按键、LED。外设中断由对应驱动处理 |
| `firmware/Core/` | 启动、向量表、链接脚本和 RTT 字节传输 |
| `firmware/Drivers/` | CMSIS-Core 头文件 |

ATIM 的内部 CH5 经 TRGO2 触发两相 ADC，ADC 中断每 20 次转换发布一组样本。电流环标称 1 kHz，位置、速度和轨迹层为 200 Hz。控制量以 mA、mV、mrad、mrad/s 为单位，电角度以编码器计数表示，正弦和余弦采用 Q14 查表。主循环消费 ADC 样本，读取 MT6701，执行控制并更新 PWM。

## 构建

需要 CMake 3.22 或更新版本、Ninja 和 Arm GNU 工具链。CMake 默认从 `PATH` 查找 `arm-none-eabi-gcc`，默认 SPL 路径为 `D:/Code/MCU/CW32/CW32L010_StandardPeripheralLib_V1.0.9`。SDK 从外部目录只读引用。

在仓库根目录运行以下命令。默认构建 Release，Debug 和 Release 分别输出到 CLion 工程的 `firmware/build/Debug/` 和 `firmware/build/Release/`，包含 ELF、HEX、BIN 和 MAP。

```powershell
.\tools\build.ps1
.\tools\build.ps1 -Configuration Debug
.\tools\build.ps1 -SdkRoot 'D:\SDK\CW32L010_StandardPeripheralLib_V1.0.9' -ToolchainRoot 'C:\ArmGNU'
```

也可直接用 CMake。自定义安装目录时，在配置命令中传入 `-DCW32_SPL_ROOT=...` 或 `-DARM_GNU_TOOLCHAIN_ROOT=...`。更换编译器安装后使用新的构建目录或重新配置干净的缓存。

```powershell
cmake -S firmware --preset CW32L010-Debug
cmake --build firmware/build/Debug
```

## 下载与调试

需要 `uv` 和 SWD 调试器。烧录与 GDB 服务使用 [pyOCD 配置](tools/pyocd/pyocd.yaml)中的 CW32L010F8 设备包，通过复位连接。接线见[硬件接口](docs/hardware/hardware-interfaces.md#swd-与复位)。

```powershell
cmake --build firmware/build/Debug --target pyocd-download
cmake --build firmware/build/Debug --target pyocd-gdbserver
```

普通构建只生成固件，`pyocd-download` 会先构建再下载。GDB 服务监听本机 3333 端口，调试时使用对应的 `CW32L010_DRV8313_FOC.elf`。

```powershell
arm-none-eabi-gdb firmware/build/Debug/CW32L010_DRV8313_FOC.elf -ex "target remote localhost:3333"
```

在 CLion 中打开 `firmware/`，在 CMake 设置中启用 `CW32L010-Debug` 或 `CW32L010-Release`。首次配置会从仓库内共享模板生成 **Build Firmware**、**Download Firmware**、**GDB Server**，这些运行配置绑定 `CW32L010-Debug`。切换到 Release 下载可用上面的 CMake 命令并指定 `firmware/build/Release`。

## 自动调试

```powershell
uv run --script tools/debug.py verify
uv run --script tools/debug.py probe
uv run --script tools/debug.py capture --seconds 3
```

`verify` 构建 Debug 和 Release 并执行 ARM ELF 回归，`probe` 枚举 SWD 调试器，`capture` 采集实机 RTT。每次运行都会留下 JSON 报告。实机烧录、限时阶跃测试和人工断点调试的用法见[调试流程](docs/debugging.md)。

## RTT CLI

烧录后保持目标运行，通过 SWD 打开一个交互会话。

```powershell
uv run --script tools/host/foc.py shell
```

在 `foc>` 中依次输入 `calibrate`、`voltage 0.2`、`enable`，即可用有感电压模式检查相序和电流方向。`status` 查看反馈，`stop` 关闭驱动。`calibrate --voltage 0.6` 可调整对齐电压，校准顺序为驱动关闭时测电流零点，施加 500 ms 对齐电压，记录编码器零点，再关闭驱动进入 `ready`。

| 命令示例 | 用途 |
| --- | --- |
| `voltage 0.2` | 有感电压，目标 0.2 V |
| `current 0.05` | 电流闭环，目标 Iq 0.05 A，Id 为 0 |
| `speed 3` | 速度闭环，目标 3 rad/s |
| `position 1.57` | 位置闭环，目标 1.57 rad |
| `impedance 1.57 0.1 0.01` | 力位，中心 1.57 rad，刚度 0.1 A/rad，阻尼 0.01 A/(rad/s) |
| `trajectory 1.57 3 8` | 轨迹定位，速度上限 3 rad/s，加速度 8 rad/s² |
| `ratchet 24 0.10 --damping 0.008` | 双向棘轮，每圈 24 格，峰值 Iq 0.10 A（可设至 0.50 A），附加阻尼 0.008 A/(rad/s) |
| `hfi 1.5` | HFI 辅助的无感运行，指令速度范围 ±1000 rad/s |
| `hfi-ramp 100` | 无感加减速斜率 100 rad/s²，可运行时修改，范围 5～1000 rad/s² |
| `bus-voltage 12` | 停机设置实际母线电压，单位 V，范围 6～24 V，0 切回采样源 |
| `fw-config 1000 950 1000` | 弱磁最大负 Id 1000 mA、电压利用率目标 950‰、负 Id 目标变化率上限 1000 mA/s，可运行时修改 |
| `gain q 0.5 30` | 调整 q 轴 PI，停机时生效，重启恢复默认值 |
| `gain sensorless-speed 0.005 0.020` | 调整 HFI/SMO 无感速度 PI，停机时生效，重启恢复此默认值 |
| `watch --csv` | 连续输出状态或 CSV，Ctrl+C 结束读取 |
| `clear` | 清除诊断事件与计数 |

支持在 PowerShell 直接执行子命令，例如 `uv run --script tools/host/foc.py status`。上电默认关闭驱动，只有 `enable` 会进入连续控制。PA3 按键和 `stop` 主动停机。编码器读失败或 ADC 暂无新样本时保留上一次输出，下一组有效样本恢复更新。采样丢拍、超时和电流偏高会显示在诊断字段中。电压、电流和有感速度命令默认范围分别为 ±6.928 V、±500 mA、±30 rad/s。电流上限用于 Iq 目标，电压模式直接输出 Vq。

HFI 注入和运行限值可在停机的 `ready` 或 `unaligned` 状态下修改。`hfi-config` 的三个参数依次为注入电压 mV、Id 励磁电流 mA、瞬时相电流过流阈值 mA。`limits` 的两个参数依次为电流矢量上限 mA 和输出电压矢量幅值上限 mV，无感模式按 Id 分配剩余 Iq 预算。电流可调上限为 2000 mA，HFI 默认瞬时过流阈值为 2000 mA。12 V 母线采用线性 SVPWM，电压矢量默认上限为 6928 mV。d/q 电压采用圆形限幅，连续 HFI 期间按剩余注入幅值预留裕量。设置仅保存在 RAM，复位后电流上限恢复默认的 500 mA，电压上限恢复 6928 mV，加速度恢复 100 rad/s²。

```powershell
uv run --script tools/host/foc.py hfi-config 200 400 2000
uv run --script tools/host/foc.py limits 2000 6928
uv run --script tools/host/foc.py hfi-ramp 100
```

手感测试在 `foc>` 中依次输入 `calibrate`、`ratchet`、`enable`，然后手动正反转电机轴。启用时的角度就是第一个齿心。转过当前齿距的 55% 时切到相邻齿心，回程阈值留有迟滞。齿心附近的回复力较软，超过 48 格时进一步减弱。齿边仍可达到设定的峰值 Iq，细格和较高峰值电流会增加基础阻尼。运行中可用 `ratchet 24 0.40 --damping 0.012` 调力度和附加阻尼，同齿数下保留当前齿位。改齿数会按当前位置重新定齿。测试后输入 `stop`。

在仓库根目录执行 `uv run --script tools/host/foc.py shell`，随后在 `foc>` 中依次执行以下命令，可设置 2 A 电流上限并启动无感转动。

```text
stop
limits 2000 6928
hfi-config 200 400 2000
smo-config enable 1
hfi-ramp 100
hfi 30
enable
```

`hfi 30` 的数值是机械 rad/s，命令范围为 ±1000 rad/s。使能时先测电流零点并两段对齐，电机轴会移动少许。锁定后旋转场默认按 100 rad/s² 的斜坡跟随目标速度，满足反电动势条件后自动交给 SMO，`status` 或 `watch` 中的 `source` 依次显示 `hfi`、`to_smo`、`smo`。625 Hz 启动阶段参考速度最高为 100 rad/s，进入 5 kHz 路径后继续跟随更高目标。SMO 完成交接、参考速度达到 40 rad/s 且估速达到 30 rad/s 后，退出启动正 Id 再转入 5 kHz 独立观测／电流环，参考达到 100 rad/s 时允许提前切入快环。启动正 Id 按 625 mA/s 退出，诊断输出仍为 500 Hz。快路径在估速降至 20 rad/s 时退出，回到 HFI 的判断继续使用反电动势条件。运行中输入 `hfi-ramp 200` 可把加减速斜率调至 200 rad/s²，停机后保留，复位恢复默认值。

`rpm = rad/s × 60 / (2π)`。363 rad/s 约为 3466 rpm，400 rad/s 约为 3820 rpm，数值对应电机轴的机械转速。CLI 的 `status` 和 `watch` 同时显示 rad/s 与 rpm。无感状态中的速度由观测器估算，受控 trace 另外用 MT6701 原始角度核对实际转速。

高速快 SMO 完成交接且估速达到 200 rad/s 后，弱磁环每 8 ms 按电压裕量调整负 Id。默认参数为 `fw-config 1000 950 1000`，`fw-config 0 950 1000` 平滑退出弱磁。启动正 Id 与弱磁负 Id 共用实际 d 轴参考，Iq 预算按 `sqrt(Imax² − Id²)` 分配，2 A 设置限制总电流矢量。参数保存在 RAM，复位恢复默认值。

运行中输入 `hfi -30` 时先由 SMO 制动减速，到低速再续接 HFI、平滑过零，随后重新交给反向 SMO。`hfi 0` 减速至位置保持，`stop` 关闭驱动和注入。MT6701 记录实测位置供对照。`watch` 用 Ctrl+C 结束显示，停机使用 `stop`。

高转速运行受反电动势与母线电压影响，可在停机后用 `limits 2000 6928` 设置 2 A 电流矢量上限并使用完整线性 SVPWM 电压范围。按当前 19 mV/(rad/s) 参数，300 rad/s 对应约 5.7 V 反电动势，还需电阻与电感压降。±1000 rad/s 是命令范围，实际可达速度取决于电机参数、母线和负载。2026-11-06 的已提交基线验证了 ±60 rad/s 反转及 ±300 rad/s 稳态，400 rad/s 指令下实测约 363 rad/s。弱磁修改期间的候选固件已测到约 400 rad/s 跟踪，以及 450 rad/s 指令下约 428 rad/s。最新离线修订保留在工作区，等待实机复测后决定提交。采样率、交接与各版实测范围见[滑模观测器](docs/firmware/smo.md)。

## 母线电压

停机后输入实际供电电压，PWM 归一化、HFI／SMO 电压重建、可用电压上限与弱磁电压目标同步更新。`status` 显示 `bus` 配置值和 `Vlimit` 当前有效上限。当前默认采样源返回 12 V 占位值，`bus-voltage 12` 切换为人工配置，`bus-voltage 0` 切回采样源。状态中的 `bus_source` 区分 `stub`、`manual` 与 `adc`。

```text
stop
bus-voltage 12
limits 2000 0
```

`limits` 的电压参数为 0 时自动跟随 `Vbus / sqrt(3)`，默认采用该策略。非零值作为显式输出上限，切换母线后保留，并受当前可用电压约束。例如设置 5000 mV 后提高母线，输出仍限制为 5000 mV。配置保存在 RAM，复位恢复 12 V、500 mA 与自动电压上限。6～24 V 是固件参数与计算范围，具体板卡、电机的供电范围需结合硬件确定。

PA3 的 ADC／KEY 流程已接入。配置位于 `firmware/BSP/Inc/bus_sense.h`，当前 `BUS_SENSE_ADC_ENABLED=0`，保留原 GPIO 按键与 12 V 占位电压。硬件接好后配置上、下分压电阻并启用该开关，PA3 使用 ADC CH3、序列 2，原两相电流顺序保持不变。上下电阻暂设 100 kΩ／10 kΩ，需按实际电阻修改。

真实采样下，PA3 接近 0 V 连续两次判定 KEY 按下，按下期间保持上一次母线估计；释放采用迟滞。主循环消费 ADC 原始值、滤波并通过 `set_bus_voltage()` 更新调制、电压重建和限幅。状态另发送 `supply_raw` 与 `key_pressed`。

## 电机播放歌曲

在仓库根目录运行，文件路径加引号即可指定任意 FFmpeg 支持的本地音频，如 MP3、FLAC、WAV、M4A。首次运行由 `uv` 安装依赖，FFmpeg 随 Python 包提供，无需单独安装。主机完成解码、单声道 48 kHz 重采样和均衡，固件接收二进制采样并输出。

```powershell
uv run --script tools/host/foc.py play "D:\音乐\歌曲.flac"
uv run --script tools/host/foc.py play "ilem、洛天依 - 大氿歌.flac" --start 45 --seconds 8
uv run --script tools/host/foc.py play "D:\音乐\歌曲.mp3" --speed 15
```

默认播放至文件结束、静止发声、峰值 5.5 V、低频增益 12 dB、高频补偿 270 µs；与最后一轮停转试听设置一致。`--speed` 指定有感闭环的机械速度，范围 ±30 rad/s，0 为静止。每次播放先停机、测电流零点并对齐，再进入速度模式，结束、异常或 Ctrl+C 时发送停机。静止播放的校准阶段也会短暂移动。当前播放采用编码器控制。

`--amplitude 4` 调整峰值电压，`--bass-db 6` 调整低频增益，`--preemphasis-us 270` 调整高频补偿。音频会统一归一化峰值，因此增加低频增益会相应减少其他频段的幅度。默认使用 μ-law 8 位传输、固件还原为 16 位，传输量约 48 KB/s；`--transport pcm16` 为 96 KB/s，对 SWD 吞吐要求更高。直接 `play` 默认 SWD 8 MHz；交互方式使用 `uv run --script tools/host/foc.py --frequency 8000000 shell`，然后输入相同的 `play` 命令。

这是资源受限的实验播放功能：最后一轮 8 秒试听仍有 1 次缓冲断流，实际约 8.54 秒，低频响度也受电机结构限制。本次保留该效果。播放后输出耗时和断流次数；`--json play ...` 输出结构化结果，断流或明显时长偏差以警告报告并返回退出码 1。歌曲文件保留在本机，不纳入 Git。

## RTT 遥测

固件运行后，可在仓库根目录通过 SWD 将 RTT 上行数据采集为二进制文件。

```powershell
uv tool run --python 3.12 --managed-python --from pyocd==0.46.0 pyocd rtt --project tools/pyocd --connect attach --address 0x20000000 --size 0x1000 --log-file telemetry.bin
```

上行通道 0 输出 76 字节整数遥测帧，由上位机解码。下行位置命令为 `@P,<目标角度>;`，单位 mrad。例如 `@P,1571;` 对应 1.571 rad。上行空间不足时整帧丢弃。帧格式与接线见[硬件接口](docs/hardware/hardware-interfaces.md#swd-与-rtt)。
