# CW32L010 调试入口

- 在仓库根目录运行 `uv run --script tools/debug.py verify`。只验证已有固件时加 `--no-build`。用标准输出中的 `report` 路径查具体构建日志和回归结果。
- 用户已连接 SWD 时，直接执行本次所需的 `flash`、RTT 连接或限时运动测试，无需每次先运行 `probe` 和 `capture`。只有连接失败、目标无响应或需要定位通信问题时，才使用 `probe`、`capture` 排查。区分离线验证与实机控制结果。
- 报告和遥测保存在 `tools/build/debug/`。固件产物仍由 CLion/CMake 预设生成在 `firmware/build/`。人工操作继续用 CLion、GDB 和 `tools/host/foc.py shell`。
- 详细命令、结果字段与定位顺序见 [docs/debugging.md](docs/debugging.md)。
