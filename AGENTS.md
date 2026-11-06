# CW32L010 调试入口

- 在仓库根目录运行 `uv run --script tools/debug.py verify`。只验证已有固件时加 `--no-build`。用标准输出中的 `report` 路径查具体构建日志和回归结果。
- 实机先运行 `uv run --script tools/debug.py probe` 和 `capture --seconds 3`。区分离线检查通过、探针可见、RTT 可读和实机控制结果。实机写入用 `flash`，限时运动测试用 `bench`，根据本次任务范围执行。
- 报告和遥测保存在 `tools/build/debug/`。固件产物仍由 CLion/CMake 预设生成在 `firmware/build/`。人工操作继续用 CLion、GDB 和 `tools/host/foc.py shell`。
- 详细命令、结果字段与定位顺序见 [docs/debugging.md](docs/debugging.md)。
