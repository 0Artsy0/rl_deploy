# RL强化学习部署
当前版本已支持 Unitree Go2 机器人在 Gazebo/MuJoCo 仿真环境下的部署，同时兼容 ONNX、TorchScript 两种模型推理方式。

## 运行指令
需打开两个终端分别执行以下操作：

### 终端 1（启动仿真环境）
```bash
# 加载环境配置
source deve/setup.bash

# 选择以下任一指令启动对应仿真平台：
roslaunch sim deploy_gazebo.launch  # 启动 Gazebo 仿真
roslaunch sim deploy_mujoco.launch  # 启动 MuJoCo 仿真
```

### 终端 2（启动控制程序）
```bash
# 加载环境配置（与终端1一致）
source deve/setup.bash

# 启动部署程序，参数需与终端1的仿真平台匹配
rosrun deploy <仿真平台> <推理模型>
```

### 参数说明
| 参数位置 | 可选值 | 说明 |
|-------|-------|-------|
| 第 1 参数 | gazebo / mujoco | 需与终端 1 启动的仿真平台一致 |
| 第 2 参数 | onnx / torchscript	| 选择模型推理方式 |

## 示例1：Gazebo 仿真 + ONNX 推理
```bash
# 终端1
source deve/setup.bash
roslaunch sim deploy_gazebo.launch

# 终端2
source deve/setup.bash
rosrun deploy gazebo onnx
```

## 示例2：MuJoCo 仿真 + TorchScript 推理
```
# 终端1
source deve/setup.bash
roslaunch sim deploy_mujoco.launch

# 终端2
source deve/setup.bash
rosrun deploy mujoco torchscript
```
