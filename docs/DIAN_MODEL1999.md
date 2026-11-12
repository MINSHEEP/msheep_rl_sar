# DIAN 第1999轮：文件传输部署

本包配对 `2026-11-11_09-38-41_student227_v1_17bb150/model_1999.pt`。
模型位置 `policy/dian/student227/policy.onnx`，输入227维、输出12维，已包含观测归一化。
现有DIAN配置、RS03映射、HI91驱动与键盘状态机保持不变。

## 包含内容

`rl_sar_dian227_model1999.tar.gz` 包含源码、DIAN配置、正式ONNX、构建/启动脚本、测试及操作说明。
不含本机x86构建产物，也不含ONNX Runtime二进制依赖；RDK X5使用板上已有的ARM64运行库。
`SHA256SUMS` 覆盖模型、配置与程序源码，解压后可验证文件完整性。

本地验证结果：

- CMake构建与七项CTest通过。
- 正式模型check通过；5秒mock完成1000个控制tick，进入RL状态，无CAN/IMU访问。
- 90帧PyTorch与ONNX动作最大绝对误差 `5.37e-7`。
- C++完整观测/推理/目标处理对照：观测与动作最大绝对误差 `1.05e-6`，目标误差 `2.43e-7 rad`。
- 测试包含运动、停车超过1秒、重新运动的历史和相位变化。

这些结果证明本地软件配对正确，不代表真实步态或板端实时性已验收。

## 在远程RDK X5解压与构建

用文件传输工具把压缩包放到远程Home目录。在新目录解压，避免覆盖已有控制器：

```bash
cd ~
test ! -e rl_sar_dian227_model1999 || { echo '目录已存在，请先检查'; exit 1; }
mkdir rl_sar_dian227_model1999
tar -xzf rl_sar_dian227_model1999.tar.gz -C rl_sar_dian227_model1999
cd rl_sar_dian227_model1999
sha256sum -c SHA256SUMS

# 按现有板端目录约定，复用custom_quad中的ARM64 ONNX Runtime。
mkdir -p library/inference_runtime
cp -a ~/custom_quad/library/inference_runtime/onnxruntime library/inference_runtime/

bash scripts/build_dian.sh
./cmake_build/bin/rl_real_dian --mode check
./cmake_build/bin/rl_real_dian --mode mock --duration 5
```

如果现有ARM64运行库位于其他位置，仅修改上面的复制来源。
运行库目录需要包含 `include/`、`lib/` 和 `VERSION_NUMBER`；不能复制开发电脑的x86运行库。
构建依赖为CMake、g++、yaml-cpp、Eigen3、TBB和Python开发头文件，按现有板端环境准备。
路径由CMake在板上构建时确定，移动整个部署目录后应重新构建。

## 键盘入口

完成板端check和mock后，按既有硬件流程启动：

```bash
cd ~/rl_sar_dian227_model1999
bash scripts/can_up.sh
./cmake_build/bin/rl_real_dian --mode read
bash scripts/keyboard_control.sh
```

keyboard启动后保持失能，按次序操作：`m`使能并保持当前姿态 → `0`过渡到默认站姿 →
等待 `Standing ready` → `1`进入RL。`w/s`前后，`a/d`横移，`q/e`转向；空格归零，`k`失能，`x`退出。
按键松开不会自动停车。每个阶段的反馈、硬件映射和停机说明见 [DIAN_RDKX5.md](DIAN_RDKX5.md)。

本次只完成本地安装和打包，未连接远程、未启动真实电机。
