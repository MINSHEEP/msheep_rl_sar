# DIAN 227 维策略：部署与 SSH 键盘操作

当前部署只支持 `proprio227_v1`：45 维本体观测 × 5 帧历史 + 当前相位 2 维。
使用 rl_sar 原有的 RL、FSM、ObservationBuffer 和 ONNX Runtime；硬件为 RDK X5、四路 SocketCAN、
RS03 和串口 HIPNUC HI91。当前使用 SSH 终端键盘控制，按键无需回车；手柄作为可选入口保留。
两种交互入口复用同一状态机，不需要额外 ROS 节点或进程间转发。

代码已在开发电脑上编译并完成离线协议、观测/滤波、IMU、键盘/手柄及状态机测试。
已安装第1999轮 ONNX，并通过90帧C++/PyTorch对照、check和5秒mock。板端使能握手和实际运动尚未执行。
本模型的现成传输包和启动步骤见 [DIAN_MODEL1999.md](DIAN_MODEL1999.md)。以下保留通用安装流程。

## 1. 在开发电脑安装最终权重

在 Lab 中导出蒸馏 student，包含对应的观测归一化，固定输入 `[1,227]`、输出 `[1,12]`。
替换下面路径为最终导出文件：

```bash
cd ~/rl_sar
DIAN_POLICY_ONNX=/absolute/path/to/final/exported/policy.onnx
cp -- "$DIAN_POLICY_ONNX" policy/dian/student227/policy.onnx
bash scripts/build_dian.sh
./cmake_build/bin/rl_real_dian --mode check
./cmake_build/bin/rl_real_dian --mode mock --duration 5
```

check 只验证配置和 ONNX；mock 使用模拟反馈，不打开 CAN/IMU，不是步态验证。
`src/rl_sar/tests/zero_actor_227.onnx` 是 184 字节的未训练零输出测试图，仅供状态机测试，不能用于部署。

## 2. 首次上传到板上的独立目录

以下上传命令供执行，本次开发没有修改远程文件。新目录与 `~/custom_quad` 分开。
在开发电脑打包：

```bash
cd ~/rl_sar
tar --exclude='*/__pycache__' -czf /tmp/rl_sar_dian227.tar.gz \
  LICENSE scripts src/rl_sar policy/dian docs/DIAN_RDKX5.md
scp /tmp/rl_sar_dian227.tar.gz sunrise@192.168.1.148:/home/sunrise/
ssh -t sunrise@192.168.1.148
```

在板上首次解压并编译。已有同名目录时先检查，不覆盖未知内容：

```bash
test ! -e ~/rl_sar_dian227 || { echo '目录已存在，请先检查'; exit 1; }
mkdir ~/rl_sar_dian227
tar -xzf ~/rl_sar_dian227.tar.gz -C ~/rl_sar_dian227
cd ~/rl_sar_dian227
mkdir -p library/inference_runtime
cp -a ~/custom_quad/library/inference_runtime/onnxruntime library/inference_runtime/
bash scripts/build_dian.sh
./cmake_build/bin/rl_real_dian --mode check
```

已只读确认板上有 CMake、g++、yaml-cpp、Eigen3、TBB、Python 开发头文件，以及 ARM64 ONNX Runtime 1.22.0。
上面的复制不改动 custom_quad 的运行库；不要复制开发电脑的 x86-64 构建产物。
上游 CMake 的非ROS模式、未找到LibTorch提示不影响这个 ONNX 构建。
以后只更新权重时，先退出控制器，再复制新 ONNX 到板上 `policy/dian/student227/policy.onnx`，重新运行 check。

## 3. 启动 CAN

先退出旧控制器、点动、标定和姿态采集程序，然后在板上运行：

```bash
cd ~/rl_sar_dian227
bash scripts/can_up.sh
```

脚本核对四个 USB 物理端口，检查共享设备锁，只配置 can0–can3 为 1 Mbps、队列长度1024。
已经处于正常 UP/1 Mbps 状态的接口不会重启；需要改配置时由 `sudo ip` 请求权限。
不要使用 `sudo bash` 启动整套程序，避免设备锁文件变成其他用户所有。

实际配置命令为：

```bash
for i in 0 1 2 3; do
  sudo ip link set can$i down
  sudo ip link set can$i type can bitrate 1000000
  sudo ip link set can$i txqueuelen 1024
  sudo ip link set can$i up
done
```

常规操作使用脚本即可，不需要再重复执行这段命令。can4 是板载接口，不用于本项目。

## 4. 首次检查关节与键盘

机器人可靠支撑、电机失能时执行只读查询：

```bash
./cmake_build/bin/rl_real_dian --mode read
```

只发17类位置、速度和运行模式查询，不使能、不停止、不标零、不写参数。
输出 `motor_q`、`joint_q`、`delta_default`、`joint_dq` 与 `run_mode`。
`run_mode=0` 是MIT控制模式，不能用于判断电机是否已失能。
机械零点与URDF零点的一致性按用户确认采用，不重新标零。

可先测试当前 SSH 终端的按键接收，不打开 CAN/IMU、不加载模型：

```bash
./cmake_build/bin/rl_real_dian --mode keyboard-read
```

按 `w` 查看 vx 增加，空格清零，`x` 退出。该命令不控制电机。
若提示需要终端，使用 `ssh -t sunrise@192.168.1.148` 连接；不要用管道输入替代交互终端。

## 5. 启动键盘控制节点

在板上前台运行：

```bash
cd ~/rl_sar_dian227
bash scripts/keyboard_control.sh
```

等价命令为 `./cmake_build/bin/rl_real_dian --mode keyboard`。
启动会先检查配置并加载227维 ONNX，然后停在 **Passive / 失能**。
不需要手柄、不需要 `--arm`，也不会自动使能。按键立即生效，不按回车。

按顺序操作，每一步看到对应提示再按下一步，不要一次粘贴整串按键：

| 顺序 | 按键 | 行为与提示 |
|---|---|---|
| 1 | `m` | 执行电机模式/watchdog/限矩读回、状态和IMU检查；通过后使能，Hold保持当前实测姿态；显示 `Motors enabled` |
| 2 | `0`（数字零） | 从当前姿态约3秒过渡到默认站姿；完成显示 `Standing ready` |
| 3 | `1`（数字一） | 站立完成后启动50Hz RL，进入 `RLFSMStateRLLocomotion` |
| 4 | `w` | vx 增加0.1m/s，开始前进；第一次先按一次 |
| 5 | 空格 | 所有速度指令归零，RL仍运行零指令站立 |
| 6 | `k` | 电机失能，回Passive；可重新从 `m` 开始 |
| 7 | `x` 或 Ctrl+C | 停止并退出程序，恢复终端输入/回显 |

运动按键：

| 按键 | 指令变化 |
|---|---|
| `w` / `s` | vx 增加 / 减少0.1m/s，减少到负值后为后退 |
| `a` / `d` | vy 增加 / 减少0.05m/s，即左移 / 右移 |
| `q` / `e` | yaw增加 / 减少0.1rad/s，即左转 / 右转 |
| 空格 | vx、vy、yaw全部清零 |
| `0` | 退出RL，重新平滑过渡到默认站姿；完成后可再按 `1` |

**SSH 终端无法提供可靠的松键事件：松开 `w` 不会停止。速度会保持，空格才清零。**
长按产生系统自动重复按键，速度逐步增加至配置上限。默认 vx±0.5m/s、vy±0.2m/s、yaw±0.5rad/s。
`m`、`0`、`1`、`k` 都会清零键盘速度，避免状态切换时带入旧指令。
箭头和功能键转义序列不作为运动指令；Ctrl+Z不挂起控制进程，退出使用 `x` 或 Ctrl+C。

当前站立是关节角插值，不是倒地恢复算法。不要求精确摆成默认站姿，但需要机身可靠支撑、
关节在配置范围内，且满足当前0.25rad机身倾角限制。不会从任意侧翻姿态自动起立。

## 6. 可选手柄入口

不使用键盘时，可改用以下入口，不能与键盘节点同时运行：

```bash
./cmake_build/bin/rl_real_dian --mode gamepad-read --duration 15
./cmake_build/bin/rl_real_dian --mode gamepad
```

默认设备 `/dev/input/by-id/usb-_Controller-joystick`，可用 `--gamepad /dev/input/js0` 指定。
摇杆回中、按钮松开后：`LB+A`使能 → 松开 → `A`站立 → 等待完成 → `RB+十字键上`启动RL。
左摇杆控制平移，右摇杆控制转向；`B`清零，`LB+B`或`LB+RB`失能。
先松开LB再按RB，避免触发LB+RB失能。编号/方向在 `policy/dian/gamepad.yaml` 中配置。

## 7. 正常结束与 SSH 断开

键盘模式下：空格清零 → 等待稳定并支撑好机身 → `k`失能 → `x`退出。
手柄模式下：B清零 → 支撑好机身 → LB+B失能 → Ctrl+C退出。
电机失能后机身会失去主动支撑。CAN可保持UP，下次启动脚本会跳过正常接口。

键盘节点处理Ctrl+C、SIGTERM和SSH的SIGHUP；终端关闭或输入错误也走停止路径。
反馈超时、异常数值、电机故障等运行检查继续有效。程序被强制杀死或主机掉电时，
电机侧已设置的watchdog负责失联停机。首次板端实际行为仍需验证。

## 训练与硬件契约

网络观测逐项堆叠，从旧到新，首帧填满历史，原始动作历史初始为零：

| Python切片 | 内容 |
|---|---|
| `[0:15]` | 角速度 × 5，clip后乘0.25 |
| `[15:30]` | 投影重力 × 5 |
| `[30:45]` | vx/vy/yaw指令 × 5 |
| `[45:105]` | 关节位置减默认姿态 × 5 |
| `[105:165]` | 关节速度 × 5，clip后乘0.05 |
| `[165:225]` | 上一控制步原始策略动作 × 5 |
| `[225:227]` | 当前相位，周期0.6秒，静止指令输出 `[0,1]` |

停车期间相位时钟继续计数；不因摇杆回中而重置历史。高度、实际线速度和接触数据不进入运行链。
动作后处理保留目标限幅、`[4,8,6]` rad/s变化率和0.04s滤波；停车1秒后使用0.1速度系数。

关节顺序 FR/FL/RR/RL，每腿 J1/J2/J3。右腿默认 `[0,-0.55,0.20]`、左腿 `[0,0.55,-0.20]`，
PD=50/1，动作缩放 `[0.25,0.5,0.65]`，目标软限位分别 ±54°、±81°、±40.5°。

| 腿 | CAN | J1/J2/J3 ID | USB位置 | 方向/软件零偏 |
|---|---|---|---|---|
| FR | can1 | 5/6/7 | 1-1.4.2:1.0 | -1 / 0 |
| FL | can0 | 1/2/3 | 1-1.4.1:1.0 | -1 / 0 |
| RR | can3 | 13/14/15 | 1-1.4.4:1.0 | -1 / 0 |
| RL | can2 | 9/10/11 | 1-1.4.3:1.0 | -1 / 0 |

以上来自2026-11-11只读检查的远程 `custom_quad/config/hardware.yaml`。
RS03主机253，位置范围±12.57、速度±20rad/s、力矩编码±60Nm；输出限矩单独设25Nm。
watchdog参数采用现有部署固件的0x7028=2000，限矩参数0x700B。使能时自动写入并读回，不保存机械零点。
HI91使用 `/dev/ttyUSB0`、115200；安装旋转来自现有配置的identity，串口驱动来源见源码目录的 NOTICE。

## 文件与验证

- `policy/dian/`：唯一的DIAN硬件、手柄及227维策略配置。
- `src/rl_sar/src/rl_real_dian.cpp`、`fsm_robot/fsm_dian.hpp`：运行入口与状态机。
- `src/rl_sar/library/robot_sdk/dian/`：SocketCAN、HI91、键盘/手柄、协议与策略契约。
- `scripts/build_dian.sh`：编译并运行七项CTest。
- `scripts/can_up.sh`：板端CAN启动。
- `scripts/keyboard_control.sh`：SSH键盘控制入口，与CAN/FSM运行于同一个进程。

测试覆盖RS03协议/映射/状态检查、HI91、90帧实际训练实现的观测与动作滤波、键盘/手柄语义、实际FSM转换，以及PTY中的完整键盘流程、信号停机和终端恢复。
FSM测试使用 `tests/zero_actor_227.onnx`，不代表训练策略效果。
第1999轮正式权重已通过本地check与mock；板端自动使能握手、手柄实操和真实步态仍需首次上机验证。
