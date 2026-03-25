# GKD Navigation 2026

本项目基于 **深圳北理莫斯科大学 北极熊战队 (SMBU PolarBear)** 的开源项目进行适配与二次开发。

> **致谢：** 诚挚感谢 [SMBU-PolarBear-Robotics-Team](https://github.com/SMBU-PolarBear-Robotics-Team) 的开源分享！
>
>**原项目地址：** [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)


---

## 目录结构

| 文件/目录 | 说明 |
| :--- | :--- |
| **`src/`** | 核心源代码，包含所有的 ROS 2 功能包 |
| **`nav2_params_*.yaml`** | 导航参数配置文件 |
| **`ui_real.py`** | 功能选择ui界面的脚本 |
| **`connect_lidar.sh`** | 激光雷达相关配置脚本 |
| **`communication.zsh`** | 用于打开导航&电控通讯节点的脚本 |
| **`check_odometry.zsh`** | 用于查看当前 odometry 话题的内容 |
| **`goal_calibration.zsh`** | 启动目标点校验的脚本 |
| **`nav_commander.zsh`** | 启动决策节点的脚本 |
| **`launch.zsh`** | 启动脚本 |

---

## Quick Start

### 1. 环境准备
确保你的 **Linux** 系统已安装以下组件：
* **ROS 2** (Humble 或更高版本)
* **Nav2** 导航框架相关 (`ros-humble-nav2-bringup` 等)
* **Python 3** 及其相关 UI 依赖库

### 2. 编译与环境加载
在工作空间根目录下执行：
```sh
colcon build --symlink-install
source install/setup.zsh
```


### 3. 运行
#### 3.1 配置激光雷达
在工作空间根目录下执行：
```bash
./connect_lidar.sh
```

#### 3.2 可视化运行:
在工作空间根目录下执行：
```sh
. ./launch.zsh
```
即可打开导航功能选择ui，点击 `建图` 按钮开启 `SLAM` 建图模式

#### 3.3 运行通讯节点
在工作空间根目录下执行：
```bash
. ./communication.zsh
```
