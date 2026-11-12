# proprio227_v1 / model_1999

已安装训练运行 `2026-11-11_09-38-41_student227_v1_17bb150` 的 `model_1999.pt` 导出的 `policy.onnx`。
固定输入 `[1,227]`，输出 `[1,12]`。ONNX 包含学生观测归一化，运行时不要再次归一化。

历史按观测项从旧到新排列，动作历史保存原始策略输出，末尾是当前相位。
参数与 `DianProprioHistoryEnvCfg` 对齐：50 Hz策略、PD=50/1、0.6秒相位周期及无接触门控的目标滤波。

已验证90帧原模型与C++端观测、推理、动作处理一致性；详细数值与权重SHA256见
`model1999_validation.json`。本地七项CTest、模型check、5秒mock通过；尚未进行真实电机运行验证。

[本模型传输与启动](../../../docs/DIAN_MODEL1999.md)

[CAN、状态机与键盘完整说明](../../../docs/DIAN_RDKX5.md)
