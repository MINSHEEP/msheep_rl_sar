# DIAN：227 维蒸馏策略部署

本目录只维护 `proprio227_v1`：45 维本体观测各保留五帧历史，末尾追加当前 2 维相位。

- `base.yaml`：关节顺序、默认站姿、200 Hz 控制与 50 Hz 推理周期。
- `hardware.yaml`：真实机器狗的四路 SocketCAN 映射、HIPNUC HI91 串口与运行阈值。
- `gamepad.yaml`：原生 Linux 手柄的设备路径、轴方向和按钮编号。
- `student227/config.yaml`：观测维度、动作缩放、PD 与无接触依赖的目标滤波。
- `student227/policy.onnx`：已安装第1999轮学生模型，包含 student 观测归一化。

运行时不需要高度、实际线速度或足端接触输入，也不需要 ROS 转发。默认使用 SSH 键盘入口 `scripts/keyboard_control.sh`，手柄入口仍可选。

完整操作流程：[DIAN RDK X5 部署与 SSH 键盘操作](../../docs/DIAN_RDKX5.md)。
