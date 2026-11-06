# 调试流程

固件构建产物放在 `firmware/build/Debug/` 和 `firmware/build/Release/`。以下命令可在仓库根目录执行，使用同一套 CMake 预设和 pyOCD 配置。

## 自动构建与离线回归

```powershell
uv run --script tools/debug.py verify
uv run --script tools/debug.py verify --configuration Debug --no-build
```

`verify` 先构建，再用 Unicorn 的 Cortex-M0 模型执行实际链接的 ELF。检查覆盖复位入口、RAM 初始化、七种模式、命令应答、轨迹参考、棘轮换齿与迟滞、故障上报与恢复、PWM/ADC 配置、两相采样顺序、定点 SVPWM 和 I²C 成功、NACK、超时。它使用确定性的 ADC 和编码器输入，以及相应的寄存器模型。模拟结果用于软件回归，真实采样相位、带载响应和闭环稳定性由实机测量。

每次执行在 `tools/build/debug/` 下生成独立目录，标准输出给出同一份 `report.json`。报告包含各检查的结果、构建日志路径、ELF 等产物的 SHA-256 和工具版本。可用 `--output <目录>` 指定报告父目录。退出码为 0 表示步骤完成，1 表示执行或检查失败，130 表示中断。

## 定点热路径与内存

`FOC_Angle()` 直接读取 4097 项四分之一周期 Q14 表。HFI 和 SMO 共用 `FOC_Atan2()`，用归一化、倒数表和八分圆反正切表求角度。`FOC_Sqrt()` 用查表初值、一次整数 Newton 迭代和精确修正求平方根，供 SMO 计算反电动势幅值。四张常量表合计 14340 字节，由链接器放在 Flash 的 `.rodata` 段。

表格由以下脚本生成。`--check` 验证已提交表格是否与生成公式一致。

```powershell
uv run --script tools/generate_foc_tables.py
uv run --script tools/generate_foc_tables.py --check
```

2026-11-06 的 Release 优化结果如下。指令统计使用相同输入执行实际 ARM ELF，正余弦取代表性角度中的最大值，其余列出固定输入路径的动态指令数。

| 项目 | 优化前 | 优化后 |
| --- | --- | --- |
| 正余弦计算 | 74 条 | 23 条 |
| HFI 更新 | 314 条 | 280 条 |
| SMO 更新，反电动势不可见 | 295 条 | 216 条 |
| SMO 更新，反电动势可见 | 700 条 | 470 条 |
| Flash 占用 | 26020 字节 | 41364 字节 |
| 静态 RAM，含 RAM 执行代码 | 2520 字节 | 2392 字节 |
| 静态 RAM 加链接器预留栈 | 4056 字节 | 3928 字节 |

有限脉冲标定与连续 HFI 共用 `measurement` 联合体，节省 128 字节 RAM。每次启动先初始化对应成员，再使能采样。剩余 1704 字节 RAM 可供运行栈使用，其中链接器预留 1536 字节。

`verify` 检查完整一圈的 16384 个正余弦相位、7257 组反正切向量和 3662 个平方根输入。正余弦与四舍五入后的 Q14 值一致，反正切实测最大误差 2 个电角度计数，平方根与整数精确取整一致。回归还覆盖有限脉冲、连续 HFI、再返回有限脉冲时的缓冲区重置。

板上 RAM 测试段连续执行 16 条乘法和 16 条移位，确认乘法耗时 32 个周期。CMake 使用 `-mtune=cortex-m0plus.small-multiply`，让 GCC 按该代价选择常量乘法的移位、加减实现。

同一块板、48 MHz 配置、相同输入、屏蔽中断下，SysTick 核心时钟计时如下。每项重复 15 次并减去空测试段的 14 个周期，结果为中位数，覆盖 Flash 等待和真实乘法延迟。

| 核心函数 | 优化前 | 优化后 | 周期减少 |
| --- | --- | --- | --- |
| 正余弦，电角度计数 1000 | 223 周期 | 45 周期 | 79.8% |
| HFI 更新 | 591 周期 | 555 周期 | 6.1% |
| SMO 更新，反电动势可见 | 1797 周期 | 1592 周期 | 11.4% |

计时数据和测试段位于 `tools/build/debug/20261106-math-timing/`。中断负载与完整电机控制的验证另用正常运行时的 RTT trace。

优化后的实机测试覆盖 +15→+30 rad/s 接管和 −30→−5 rad/s 回退。两组解调消费率分别为 624.93 Hz 和 624.94 Hz，控制丢拍、解调跳帧、编码器错误与 RTT 丢记录均无新增，结束时均确认停机。

随后开放了 2000 mA 的 Iq 可调上限和默认 HFI 相电流过流阈值。HFI 每个矢量累加 38 次采样，2 A 时单项可达到 47158 个 ADC 计数，因此内部累加改用 32 位，再转换为原有 Q3 均值输出。双缓冲仍复用有限脉冲的存储区。该版本 Release Flash 为 41388 字节，静态 RAM 为 2336 字节，连同 1536 字节预留栈共 3872 字节。回归新增正负 2 A 的连续三帧及三相过流边界，shell 实机流程验证了设置参数、无感启动、接管至 SMO 和停机。

## 实机取证与限时测试

| 命令 | 行为 |
| --- | --- |
| `uv run --script tools/debug.py probe` | 枚举 SWD 调试器和 ID |
| `uv run --script tools/debug.py capture --seconds 3` | 附加并读取 RTT 上行状态，保存 `telemetry.jsonl`、丢拍、编码器错误和当前状态。若目标已暂停，报告记录 PC、SP、LR |
| `uv run --script tools/debug.py trace --seconds 5 --divider 2` | 以标称 500 Hz 读取原始角度和电流采样，保存 `trace.jsonl`，不改变当前控制模式 |
| `uv run --script tools/debug.py trace --speed=-10 --seconds 5` | 校准后以 −10 rad/s 运行，等待稳态后采集，结束时确认停机 |
| `uv run --script tools/debug.py hfi --amplitude 0.2 --periods 2` | 静止时发成对短脉冲，保存零注入基线、两相 ADC 响应及 MT6701 电角度 |
| `uv run --script tools/debug.py hfi --amplitude 1.5 --periods 0 --repeats 48 --points 48 --step 0.12 --reverse` | 测量静止位置的连续 25 kHz 旋转载波响应，供 HFI 估角标定 |
| `uv run --script tools/debug.py flash --configuration Release` | 烧录现有 ELF，记录烧录日志和产物哈希 |
| `uv run --script tools/debug.py bench --mode current --amplitude 0.05 --seconds 0.5` | 校准后依次测零指令、正阶跃、回零、负阶跃和回零。每段采集 0.5 秒，最后发 `stop` 并记录应答 |
| `uv run --script tools/debug.py bench --mode hfi --amplitude 1.5 --seconds 6` | HFI 锁定后测试正反转，每阶段 6 秒，无感命令支持至 ±1000 rad/s。用 MT6701 位置独立统计约 200 ms 窗口的速度和停顿次数 |

存在多个探针时，实机命令用 `--probe <ID>` 选定目标。常规接手顺序是先运行 `verify`，再用 `probe` 和 `capture` 查看已连接目标。烧录和带电机运动的试验分别由 `flash` 和 `bench` 执行。做电压检查时可用 `bench --mode voltage --amplitude 0.2`。阶跃报告的 `ok` 表示流程完成，电流跟踪、方向、过冲和稳定性通过各阶段遥测评估。

诊断时先看 `sample_us` 是否接近 1000，以及 `missed_samples_delta`、`encoder_errors_delta` 是否增长。再结合 `iq_ref_ma`、`iq_ma` 和 `vq_mv` 区分设定值、实际反馈和电压输出。限时试验结束时以报告的 `final_stop` 为准；若 SWD 通信中断，板上 PA3 可用于主动停机。

`trace` 默认 SWD 时钟为 2 MHz。`--divider 1` 尝试每个控制样本都记录，`--divider 2` 每两次记录一次。`index` 检查 RTT 是否丢记录，`sequence` 和 `time_us` 检查控制采样间隔。`raw_angle` 是 MT6701 的原始 14 位计数，换算到控制方向时取反并解回绕。HFI 运行时 `hfi_angle` 和 `hfi_speed_mrad_s` 记录 HFI 估算角度和速度。任一角度字段为 65535 表示该角度无效。控制量沿用状态帧单位。报告同时给出固件测速、HFI 估速波动、估角反向/大步跳变、用原始角度差分重算的测速标准差和 Iq 跟踪均方根误差。

`hfi --points 48` 会在多个静止位置测量，试验点之间会低速转动电机。`--reverse` 改为反向取点，`--step` 设置相邻点的最小机械位移。每个点先测零注入，再沿 +α、−α、+β、−β 施加 1–4 个 PWM 周期的短脉冲，各重复最多 64 次。MCU 在每次写 PWM 后采下一组原始 ADC，并记录切回零矢量后的一个样本，脉冲完成时关闭驱动。`hfi.jsonl` 保存两相 ADC 相对脉冲前基线的和、平方和及重复次数。报告剔除测量期间转动超过 4 mrad 或出现传感器错误的点，用一半角度点拟合二倍电角度响应，在另一半上报告轴线估角误差。

`--periods 0` 改用连续 25 kHz 旋转载波，PWM 同步提高到 100 kHz。每个载波周期轮换 +α、+β、−α、−β。HFI 电机模式的电流环与角度观测每 40 个载波周期更新一次，计算率为 625 Hz。每个矢量再次输出时，以 25 kHz 对对应占空比做一阶平滑，每次收敛剩余误差的 1/8。目标和滤波状态使用 Q8 定点数，写入 PWM 时才取整，减少电流环帧边界的电压台阶。诊断命令的有限次数连续注入维持原始矢量输出。停机后恢复 20 kHz PWM。

普通模式的前台 ADC 样本以八项队列按 1 kHz 保存。连续 HFI 模式及其快 SMO 阶段的诊断队列均为 500 Hz。队列溢出时保留最近八项，返回序号的跳变仍计入 `missed_samples`。前台先处理待消费的无感控制帧，再进行编码器诊断与 RTT 输出。trace 的 `hfi_sequence` 沿用为无感控制帧序号，`hfi_missed_frames` 累计本次使能后的跳帧数。低速 `hfi_processed_frames_hz` 应接近 625 Hz，快 SMO 稳态应接近 5000 Hz，跨采样率采集时为区间平均。`smo_sample_us` 分别为 1600 和 200。受控 `trace --mode hfi --speed ...` 在前台样本丢失或控制帧跳帧时返回失败，并保存停机结果。

高频中断直接持有当前累加缓冲区的指针，避免每次采样计算 44 字节数组步长。连续 HFI 处理函数独立放在 RAM 中，减少中断入口的寄存器占用。ADC 队列读端使用发布序号检查，不关闭全局中断。前台每轮处理一次控制帧，再服务诊断和命令，避免一轮连续消费两个控制帧挤占诊断队列。

快 SMO 的 MT6701 诊断使用非阻塞 I²C 状态机，每轮只处理一个已经就绪的总线状态，控制帧可在字节之间继续更新。诊断结果就绪后再发送对应 trace。采样率切换期间先结束已有事务，再消费下一项诊断，避免阻塞读与非阻塞读交叉。停机原因保留到清诊断，随后出现的采样周期变化不会覆盖过流或失锁事件。

RTT 的上行缓冲区按 4 字节对齐，状态包和 trace 按连续区间做字拷贝，再发布写指针。跨环尾分为两次拷贝，非对齐输入及尾部字节逐字节处理。2026-11-06 的反转复测中，原有逐字节环形写入与低速控制、编码器读取叠加，丢拍以约 40 ms 的状态上报间隔出现。改动后同版固件连续三轮 ±60 rad/s 反转均无控制丢拍、诊断丢样、编码器错误或 RTT 丢记录。`verify` 对两条上行通道执行 128 组对齐、非对齐、跨环尾、零长度与满缓冲区用例。

快 SMO 的入口、出口与反转策略及本轮实机范围见[滑模观测器](firmware/smo.md#2026-11-06-实机回归)。`verify` 包含 12.02 秒的应用入口回放，使用理想电流和观测输入检查正转到反转的实际命令路径，另以 ADC 寄存器模型检查四次 20 kHz 转换形成一个 5 kHz 帧、500 Hz 诊断队列及 PWM 频率切换。

无感参考加减速度默认 100 rad/s²，可在 shell 中运行 `hfi-ramp 200` 调整，允许 5～1000 rad/s²，运行中立即生效。命令速度、CLI `bench` 和 HFI `trace` 范围统一为 ±1000 rad/s。有感 `speed` 与对应 `trace` 仍为 ±30 rad/s。12 V 母线的默认电压矢量上限为 6928 mV，停机时运行 `limits 2000 6928` 可同时将 Iq 上限设为 2 A。

## 人工调试

SMO 旁路观测的参数、反电动势交接判据和实机结果见 [滑模观测器](firmware/smo.md)。`trace --mode hfi --speed 15` 用连续注入模式采集，`trace --speed 15` 用编码器速度控制采集。trace 帧为 72 字节，主机 CLI 应与固件一同更新。固件发送 `smo_speed_q8` 和 `smo_sample_us`，CLI 换算出 `smo_speed_mrad_s`，采集热路径直接发送原始定点值。`--require-smo` 要求采集末尾至少连续 1 秒保持 SMO 控制。

CLion 打开 `firmware/` 后可使用 Build Firmware、Download Firmware 和 GDB Server。软件断点和变量检查用 GDB，连续运行时用 RTT shell。两者使用同一个 SWD 探针时，先结束当前连接再切换。

```powershell
uv run --script tools/host/foc.py shell
uv run --script tools/host/foc.py --json status
uv run --script tools/host/foc.py --json watch --seconds 3
```

交互命令与单位见 [RTT CLI](../README.md#rtt-cli)。`--json status` 返回一条结构化状态，`--json watch` 每行返回一帧，便于脚本或后续 GUI 消费。人工采集可用 `watch --csv`。
