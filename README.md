# RM Vision

面向 RoboMaster 的 ROS 2 视觉自瞄系统。项目集成工业相机驱动、装甲板检测与数字分类、目标跟踪、云台坐标系描述和串口通信，可在真机上完整启动，也可通过外部图像话题进行无硬件调试。

<p align="center">
  <img src="rm_vision/docs/rm_vision.svg" alt="RM Vision" width="220">
</p>

> 当前项目以 **Ubuntu 22.04 + ROS 2 Humble** 为主要运行环境。

## 功能

- 支持迈德威视（MindVision）和海康机器人（HikRobot）工业相机
- 基于灯条特征完成装甲板检测，并通过 MLP 识别装甲板数字
- 使用 PnP 解算目标在相机坐标系中的三维位姿
- 使用扩展卡尔曼滤波器（EKF）估计目标状态
- 通过 TF 维护云台、相机与惯性坐标系之间的关系
- 通过串口接收云台姿态与目标颜色，并向电控发送跟踪结果
- 提供检测图像、Marker 和跟踪状态等调试话题

## 系统流程

```text
工业相机
   │  /image_raw, /camera_info
   ▼
armor_detector ── /detector/armors ──▶ armor_tracker
                                                │
                                      /tracker/target
                                                ▼
                                      rm_serial_driver
                                                │
                                                ▼
                                               电控
```

串口节点同时发布云台姿态对应的 TF，并根据电控数据动态设置敌方颜色。`robot_state_publisher` 负责发布云台到相机的静态坐标变换。

## 仓库结构

| 目录 | 说明 |
| --- | --- |
| `rm_vision/rm_vision_bringup` | 整套系统的 launch 与公共参数 |
| `rm_auto_aim/armor_detector` | 装甲板检测、数字分类与 PnP 解算 |
| `rm_auto_aim/armor_tracker` | 目标匹配与 EKF 跟踪 |
| `rm_auto_aim/auto_aim_interfaces` | 自定义 ROS 2 消息 |
| `rm_serial_driver` | 视觉与电控之间的串口通信 |
| `rm_gimbal_description` | 云台与相机坐标系的 URDF/Xacro 描述 |
| `ros2_mindvision_camera` | 迈德威视相机 ROS 2 驱动 |
| `ros2_hik_camera` | 海康机器人相机 ROS 2 驱动 |
| `rm_vision_simulator` | Unity 仿真工程 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble Desktop
- 支持 C++14 的编译器
- OpenCV、Eigen3 与 ROS 2 开发依赖
- MindVision 或 HikRobot 相机及对应平台的 SDK 动态库（真机运行时）

仓库中的相机驱动会按主机架构从各自的 `mvsdk/lib/<arch>` 或 `hikSDK/lib/<arch>` 目录链接厂商库。请在编译前确认所用 SDK 动态库存在且与 `x86_64`/`aarch64` 架构匹配；当前仓库仅包含海康 SDK 的 `arm64` 动态库，其他平台需要自行补充厂商 SDK。

## 编译

将仓库克隆到 ROS 2 工作空间的 `src` 目录：

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/liangbeisong/rm_vision2025.git rm_vision
cd ~/ros_ws
```

安装依赖并编译：

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init        # 本机首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果 `rosdep init` 提示已经初始化，可直接跳过该命令。串口模块依赖 ROS 2 的 `serial_driver`，也可单独安装：

```bash
sudo apt install ros-humble-serial-driver
```

## 配置

启动前主要检查以下文件：

### 启动配置

编辑 `rm_vision/rm_vision_bringup/config/launch_params.yaml`：

```yaml
camera: mv  # mv: MindVision，hik: HikRobot

odom2camera:
  xyz: "\"0.10 0.0 0.05\""
  rpy: "\"0.0 0.0 0.0\""
```

`odom2camera` 用于描述相机相对云台惯性系的安装位置和姿态，长度单位为米，角度单位为弧度。

### 节点参数

编辑 `rm_vision/rm_vision_bringup/config/node_params.yaml`：

- `/camera_node`：曝光时间、增益、图像翻转与标定文件
- `/serial_driver`：串口设备、波特率和时间戳偏移
- `/armor_detector`：敌方颜色、二值化阈值与分类置信度
- `/armor_tracker`：目标坐标系、匹配阈值与 EKF 噪声参数

其中 `detect_color` 的含义为：

| 值 | 识别颜色 |
| --- | --- |
| `0` | 红色 |
| `1` | 蓝色 |

真机运行时，串口接收到的颜色会动态覆盖该参数。

### 相机标定

将标定结果写入 `rm_vision/rm_vision_bringup/config/camera_info.yaml`。可使用 ROS 2 相机标定工具：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/image_raw camera:=/
```

请根据实际标定板修改内角点数量和方格边长。

### 串口权限

默认设备为 `/dev/ttyACM0`。若当前用户无访问权限，可将其加入 `dialout` 组，重新登录后生效：

```bash
sudo usermod -aG dialout "$USER"
```

## 运行

### 完整系统

在工作空间根目录执行：

```bash
./src/rm_vision/run_vision.sh
```

也可以手动启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

完整启动会运行相机、检测器、跟踪器、串口驱动和 `robot_state_publisher`。任一核心进程退出后，launch 会结束整套系统。

### 无硬件调试

不启动相机和串口，仅运行检测、跟踪与坐标系节点：

```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

此模式需要由相机、rosbag 或仿真器提供以下话题：

- `/image_raw`：`sensor_msgs/msg/Image`
- `/camera_info`：`sensor_msgs/msg/CameraInfo`

例如播放已有 rosbag：

```bash
ros2 bag play <bag_directory>
```

## 常用话题

| 话题 | 类型/用途 |
| --- | --- |
| `/image_raw` | 相机原始图像 |
| `/camera_info` | 相机内参与畸变参数 |
| `/detector/armors` | 当前帧检测到的装甲板 |
| `/tracker/target` | 跟踪器输出的目标状态 |
| `/detector/result_img` | 带识别结果的调试图像 |
| `/detector/binary_img` | 二值化调试图像 |
| `/detector/marker` | 检测结果 RViz Marker |
| `/tracker/marker` | 跟踪结果 RViz Marker |
| `/aiming_point` | 电控回传瞄准点 Marker |
| `/latency` | 视觉链路延迟 |

调试图像仅在 `node_params.yaml` 中启用 `armor_detector.debug` 后发布。可使用以下命令查看：

```bash
ros2 run rqt_image_view rqt_image_view
```

## 测试

```bash
colcon test --packages-up-to rm_auto_aim
colcon test-result --verbose
```

## 常见问题

### 找不到相机 SDK 动态库

确认动态库已经放入对应相机驱动的 `lib/amd64` 或 `lib/arm64` 目录，并与当前系统架构一致。必要时将厂商库所在目录加入 `LD_LIBRARY_PATH`，然后重新编译。

### 无法打开串口

检查设备名是否正确、用户是否属于 `dialout` 组，以及串口是否被其他进程占用：

```bash
ls -l /dev/ttyACM*
groups
```

### 有图像但没有检测结果

依次确认相机标定文件有效、`detect_color` 与目标颜色一致、曝光不过亮或过暗，并根据现场光照调整 `exposure_time`、`gain` 和 `binary_thres`。

### 跟踪器没有输出

检查 `/detector/armors` 是否有数据，以及 `odom`、`camera_link` 等 TF 是否完整：

```bash
ros2 topic hz /detector/armors
ros2 run tf2_ros tf2_echo odom camera_link
```

## 相关文档

- [装甲板检测](rm_auto_aim/armor_detector/README.md)
- [目标跟踪](rm_auto_aim/armor_tracker/README.md)
- [串口通信](rm_serial_driver/README.md)
- [云台坐标系](rm_gimbal_description/README.md)
- [MindVision 相机](ros2_mindvision_camera/README.md)
- [HikRobot 相机](ros2_hik_camera/README.md)

## 许可证

各子模块的许可证可能不同，请以对应目录中的 `LICENSE` 和 `package.xml` 为准。厂商相机 SDK 及其动态库遵循各厂商的授权条款。
