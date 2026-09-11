# 问题4演练验证报告

- Claim：Jammers Simulator v1.1 问题4演练案例 `DAPN-DCHQ-YAZ8-V2QU` 已由 v4 策略正常执行并退出，实际 14 个干扰源全部清除。
- Command：`python practice_robot_v4.py --problem 4 --mode practice --label q4_DAPN-DCHQ-YAZ8-V2QU --out results_v4`
- Executed timestamp：2026-09-11 14:00:42 至 14:01:11（Asia/Shanghai）
- Exit code：0
- Output summary：清除 14 个频道；证书排除 6 个无源频道；访问 31/31 个覆盖站；362 次请求；虚拟时间 9588.50682 秒；平均 684.893344 秒/源；移动 37512.534 米；检测 340 次；切频 298 次；失败清除 6 次；尾段确认 1136.81112 秒。
- Warnings：本次为单个随机演练案例，不能据此证明普遍成功率或与历史不同案例之间的固定提速比例；移动仍占主要成本，尾段确认占总虚拟时间约 11.86%。
- Errors：无。机器人记录中 362 次请求全部 HTTP 200 且 `accepted=true`；首条 `/enter`、末条 `/exit`；计时对账误差 1.9439e-6 秒。
- Verdict：PASS

## 模拟器侧真值

模拟器生成的 `practice-p4-4946629966119462907-DAPN-DCHQ-YAZ8-V2QU.result.json` 显示：

- `problem_no=4`
- `jammer_count=14`
- `omnidirectional_jammer_count=10`
- `directional_jammer_count=4`
- 案例编码与本次运行一致

因此本次清除率为 14/14，即 100%。

## Automated Coverage

- Support detected：存在本地机器人 CLI 与模拟器 HTTP 接口，没有独立 E2E 测试框架。
- Required flow：演练前置界面校验、`/enter`、移动/检测/切频/清除、覆盖与无源证书、`/exit`、模拟器真值回读。
- Classification：`manual-only`（真实随机演练需要用户从软件界面启动；机器人执行及结果核验已自动化）。
- Specs added or updated：无。
- Commands executed：v4 问题4演练命令；请求日志审计；模拟器 `.result.json` 真值回读；SHA-256 计算。
- Blocked items：桌面控制接口未暴露此原生 Wails/WebView2 窗口，因此开始演练由用户手动点击；不影响本次机器人链路和结果核验。

## Evidence

- `results_v4/q4_DAPN-DCHQ-YAZ8-V2QU.jsonl`
- `results_v4/q4_DAPN-DCHQ-YAZ8-V2QU_summary.json`
- `results_v4/q4_DAPN-DCHQ-YAZ8-V2QU_certificate.json`
- 模拟器 `JammersSimulatorData/behavior-logs/practice-p4-4946629966119462907-DAPN-DCHQ-YAZ8-V2QU.result.json`

校验值：

- 请求日志：`1B708D38C63D184700C048EF2397D70C84CCADE1D8CCCB599AB2548D2EF536B3`
- 汇总：`948CAB90D149BBE693EE308762C0B922C192B8365CA6D62D498C3EFA41EFC1D8`
- 证书：`1C9C1B72C2BB3CC1A34D1431F271774F7493297704B013E93D1BD9E3F046B849`
