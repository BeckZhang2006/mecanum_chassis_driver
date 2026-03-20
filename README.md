# Mecanum Chassis Driver

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

ROS2 Jazzy 四驱麦轮（Mecanum）底盘驱动节点，支持速度控制、里程计发布和TF变换。

## 功能特性

- ✅ ROS2 Jazzy 兼容
- ✅ 标准麦轮运动学（逆/正运动学）
- ✅ 实时编码器数据接收与里程计计算
- ✅ TF 变换发布（odom → base_link）
- ✅ 串口通信（自动数据包解析）
- ✅ 编码器溢出处理
- ✅ 线程安全设计

## 坐标系定义

遵循 ROS2 标准坐标系：

| 轴 | 方向 | 正方向 |
|:---:|:---:|:---:|
| X | 前后 | 向前为正 |
| Y | 左右 | 向左为正 |
| Z | 上下 | 向上为正 |
| 旋转 | 偏航 | 逆时针为正（从上方看）|

## 电机布局

```
前
    M1(左前) ─────── M3(右前)
      │                │
      │      底盘      │
      │                │
    M2(左后) ─────── M4(右后)
后
```

## 编译

### 前提条件

- ROS2 Jazzy Jalisco 已安装
- Python 3.10+
- `pyserial` 库

### 安装依赖

```bash
# 安装 ROS2 依赖
sudo apt update
sudo apt install -y python3-pip python3-serial ros-jazzy-tf2-ros

# 创建工作空间（如果还没有）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆或复制本包到 src 目录
git clone <repository_url>  # 或手动复制
```

### 编译

```bash
cd ~/ros2_ws

# 安装 rosdep 依赖（首次）
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mecanum_chassis_driver

# 或全量编译
colcon build

# 更新环境
source install/setup.bash
```

## 运行测试

### 单元测试

```bash
cd ~/ros2_ws
source install/setup.bash

# 运行所有测试
colcon test --packages-select mecanum_chassis_driver

# 查看测试结果
colcon test-result --verbose
```

### 代码检查

```bash
# 版权检查
ros2 run mecanum_chassis_driver test_copyright.py

# 代码风格检查 (flake8)
ros2 run mecanum_chassis_driver test_flake8.py

# 文档字符串检查 (pep257)
ros2 run mecanum_chassis_driver test_pep257.py
```

## 使用

### 1. 配置串口权限

```bash
# 添加用户到 dialout 组（免 sudo 使用串口）
sudo usermod -a -G dialout $USER

# 重新登录或执行
newgrp dialout

# 查看串口设备
ls -la /dev/ttyUSB*
```

### 2. 启动驱动节点

#### 方式一：使用 Launch 文件（推荐）

```bash
# 使用默认配置
ros2 launch mecanum_chassis_driver chassis_driver.launch.py

# 指定串口
ros2 launch mecanum_chassis_driver chassis_driver.launch.py port:=/dev/ttyUSB0

# 指定自定义配置文件
ros2 launch mecanum_chassis_driver chassis_driver.launch.py config:=/path/to/custom_config.yaml
```

#### 方式二：直接运行节点

```bash
# 使用默认参数
ros2 run mecanum_chassis_driver chassis_driver

# 指定参数
ros2 run mecanum_chassis_driver chassis_driver --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baudrate:=115200 \
    -p wheel_diameter:=0.067
```

### 3. 测试控制

#### 发布速度指令

```bash
# 前进 (X轴 0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 平移 (Y轴 0.3 m/s，向左)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 旋转 (Z轴 1.0 rad/s，逆时针)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'

# 组合运动
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

#### 使用键盘控制

```bash
# 安装键盘控制包
sudo apt install ros-jazzy-teleop-twist-keyboard

# 运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. 查看输出

```bash
# 查看里程计数据
ros2 topic echo /odom

# 查看轮子速度反馈
ros2 topic echo /wheel_speeds

# 查看 TF 变换
ros2 run tf2_ros tf2_echo odom base_link

# 查看所有话题
ros2 topic list

# 查看节点图
ros2 run rqt_graph rqt_graph

# 使用 RViz2 可视化
ros2 run rviz2 rviz2
```

## 配置参数

配置文件位置：`config/chassis_config.yaml`

```yaml
mecanum_chassis_driver:
  ros__parameters:
    port: '/dev/ttyUSB0'           # 串口设备
    baudrate: 115200               # 波特率
    wheel_diameter: 0.067          # 轮子直径 67mm
    wheel_base: 0.15               # 半轮距（左右轮间距的一半，单位米）
    axle_base: 0.15                # 半轴距（前后轮间距的一半，单位米）
    encoder_resolution: 1560       # 编码器分辨率（线数×减速比）
    control_rate: 50.0             # 控制频率 Hz
    odom_rate: 50.0                # 里程计发布频率 Hz
    speed_scale: 1000.0            # 速度比例：1000指令值对应 1.0 m/s
    upload_encoder: true           # 开启编码器数据自动上报
    encoder_max: 2147483647        # 编码器最大值（溢出检测）
    encoder_min: -2147483648       # 编码器最小值
```

## 运动学公式

### 逆运动学（给定底盘速度 → 计算各轮速度）

```
v1(M1/左前) = Vx - Vy - (L+W) * Ω
v2(M2/左后) = Vx + Vy - (L+W) * Ω
v3(M3/右前) = Vx + Vy + (L+W) * Ω
v4(M4/右后) = Vx - Vy + (L+W) * Ω
```

其中：
- `Vx`：X轴速度（向前为正）
- `Vy`：Y轴速度（向左为正）
- `Ω`：角速度（逆时针为正）
- `L`：半轮距
- `W`：半轴距

### 正运动学（给定各轮位移 → 计算底盘位移）

```
dx     = (d1 + d2 + d3 + d4) / 4
dy     = (-d1 + d2 - d3 + d4) / 4
dtheta = (-d1 - d2 + d3 + d4) / (4 * (L+W))
```

## 通信协议

驱动板与 ROS 节点通过串口通信，数据包格式：

### 发送指令

| 指令 | 格式 | 说明 |
|:---:|:---|:---|
| 速度控制 | `$spd:v1,v2,v3,v4#` | v1-v4 范围 -1000~1000 |
| 轮子直径 | `$wdiameter:67#` | 单位 mm |
| 编码器上报 | `$upload:1,1,1#` | 总编码器,实时编码器,速度 |

### 接收数据

| 数据类型 | 格式 | 说明 |
|:---:|:---|:---|
| 总编码器 | `$MALL:M1,M2,M3,M4#` | 累计编码器值 |
| 实时编码器 | `$MTEP:M1,M2,M3,M4#` | 瞬时编码器值 |
| 轮子速度 | `$MSPD:v1,v2,v3,v4#` | 轮子转速 |
| 电池电量 | `$Battery:7.40V#` | 电池电压 |

## 常见问题

### Q: 串口打开失败

```bash
# 检查串口设备是否存在
ls -la /dev/ttyUSB*

# 检查权限
sudo usermod -a -G dialout $USER

# 或临时使用 sudo（不推荐）
sudo chmod 666 /dev/ttyUSB0
```

### Q: 里程计漂移严重

- 检查 `wheel_diameter`、`wheel_base`、`axle_base` 参数是否与实际一致
- 检查 `encoder_resolution` 是否正确（编码器线数 × 减速比）
- 确认轮子安装方向正确（麦轮辊子呈 45°）

### Q: 节点启动后立即退出

```bash
# 查看详细日志
ros2 launch mecanum_chassis_driver chassis_driver.launch.py 2>&1 | tee log.txt

# 检查依赖
ros2 pkg list | grep mecanum_chassis_driver
```

### Q: TF 变换异常

```bash
# 检查 TF 发布
ros2 topic echo /tf

# 检查静态 TF
ros2 topic echo /tf_static

# 使用 tf2 工具检查
ros2 run tf2_tools view_frames.py
```

## 开发

### 包结构

```
mecanum_chassis_driver/
├── config/
│   └── chassis_config.yaml     # 配置文件
├── launch/
│   └── chassis_driver.launch.py  # Launch 文件
├── mecanum_chassis_driver/
│   ├── __init__.py
│   └── chassis_driver.py       # 主节点代码
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── README.md
```

### 调试

```bash
# 设置日志级别为 DEBUG
ros2 run mecanum_chassis_driver chassis_driver --ros-args \
    --log-level debug
```

## 许可

MIT License

## 致谢

- [ROS2](https://docs.ros.org/en/jazzy/)
- [麦轮运动学参考](http://robotsforroboticists.com/drive-kinematics/)
