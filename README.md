# Muse Pi Pro（RISC-V）上完整部署 ROS2 RTK

## 介绍
Muse Pi Pro（RISC-V）上完整部署 ROS2 RTK，RTK ROS 驱动包提供全面的传感器数据采集和机器人控制功能。这是一个集成驱动程序，同时处理 IMU 传感器、GPS RTK 定位数据和机器人运动控制。。这是一个集成驱动程序，同时处理 IMU 传感器、GPS RTK 定位数据和机器人运动控制。

[https://github.com/Abner2017/Muse-Pi-Pro-ROS-RTK/blob/main/%E6%9E%B6%E6%9E%84.png]

## 功能特性
- **IMU 数据**: 200Hz高频率惯性测量单元数据发布
- **GPS RTK**: 实时动态 GPS 定位，提供厘米级精度
- **机器人控制**: 集成的速度命令接口
- **电池监控**: 实时电池电压监测
- **阻塞式串口**: 高效的数据读取，无需定时器

## 软件架构
```
┌─────────────────────────────────────────────┐
│         Muse Pi Pro-RTK_driver_node               │
│                                             │
│  ┌─────────────┐  ┌─────────────────────┐   │
│  │  IMU 解析器 │  │    GPS 解析器         │   │
│  │   (200Hz)   │  │  (经纬度+高度+速度)   │   │
│  └─────────────┘  └─────────────────────┘   │
│  ┌─────────────┐  ┌─────────────────────┐   │
│  │  电池监控   │  │   机器人控制器         │   │
│  │    监控     │  │  (AT+CTRL指令)       │   │
│  └─────────────┘  └─────────────────────┘  │
│  ┌─────────────────────────────────────────│─┐
│  │         阻塞式串口读取 (/dev/ttyACM2)     │
│  └─────────────────────────────────────────│─┘
└─────────────────────────────────────────────┘
         │
         ▼
    ROS 话题发布
   /fbrtk/imu/data (sensor_msgs/Imu)
   /fbrtk/gps/fix (sensor_msgs/NavSatFix)
   /fbrtk/velocity (geometry_msgs/TwistStamped)
   /fbrtk/battery_voltage (std_msgs/Float32)
   
   订阅 /cmd_vel (geometry_msgs/Twist)
```

## 安装教程

### 依赖要求
- ROS Noetic (Ubuntu 20.04) 或 ROS Melodic (Ubuntu 18.04)
- catkin 工作空间
- 串口访问权限

### 编译安装
1. 克隆仓库到 catkin 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```

2. 安装依赖：
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-nav-msgs
   sudo apt install ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs
   sudo apt install libboost-system-dev
   ```

3. 编译包：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. 设置串口权限：
   ```bash
   sudo chmod 777 /dev/ttyACM2
   ```

## 使用说明

### 硬件设置
1. 将 FB-RTK 设备连接到 USB 端口（通常是 `/dev/ttyACM2`）
2. 验证设备连接：
   ```bash
   ls -l /dev/ttyACM*
   ```

### 基本启动
1. 启动 FB-RTK 集成驱动：
   ```bash
   roslaunch fbrtkros fbrtk_driver.launch
   ```

2. 带可视化启动：
   ```bash
   roslaunch fbrtkros fbrtk_visualization.launch
   ```

3. 测试键盘控制：
   ```bash
   # 5Hz频率发送/cmd_vel命令
   python3 /home/chenmingjie/catkin_ws/src/FB-RTK/fb-rtkros/scripts/simple_control.py
   ```

### 配置参数
编辑启动文件参数以匹配您的设置：
```xml
<!-- 串口配置（固定FB-RTK设备） -->
<param name="port" value="/dev/ttyACM2" />
<param name="baudrate" value="115200" />

<!-- 速度限制 -->
<param name="max_linear_velocity" value="1.0" />
<param name="max_angular_velocity" value="1.0" />
```

### 发布话题
- `/fbrtk/imu/data` (sensor_msgs/Imu): IMU 测量数据，约200Hz
- `/fbrtk/gps/fix` (sensor_msgs/NavSatFix): GPS 位置信息
- `/fbrtk/velocity` (geometry_msgs/TwistStamped): GPS 速度信息
- `/fbrtk/battery_voltage` (std_msgs/Float32): 电池电压

### 订阅话题
- `/cmd_vel` (geometry_msgs/Twist): 速度命令

### 机器人控制
使用键盘控制脚本（推荐）：
```bash
python3 /home/chenmingjie/catkin_ws/src/FB-RTK/fb-rtkros/scripts/simple_control.py
```

或直接发送速度命令：
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

### 监控数据
监控传感器数据：
```bash
# IMU 数据（约200Hz）
rostopic echo /fbrtk/imu/data

# GPS 数据
rostopic echo /fbrtk/gps/fix

# GPS 速度
rostopic echo /fbrtk/velocity

# 电池电压
rostopic echo /fbrtk/battery_voltage

# 检查数据频率
rostopic hz /fbrtk/imu/data
```

## 故障排除

### 常见问题
1. **串口权限被拒绝**：
   ```bash
   sudo chmod 666 /dev/ttyACM2
   ```

2. **传感器无数据**：
   - 检查 `/dev/ttyACM2` 是否存在
   - 验证波特率设置 (115200)
   - 检查设备电源和连接

3. **机器人不响应命令**：
   - 检查串口连接
   - 验证AT+CTRL命令格式
   - 监控/cmd_vel话题是否有数据

4. **IMU数据频率异常**：
   - 检查串口通信稳定性
   - 使用 `rostopic hz /fbrtk/imu/data` 监控频率

### 调试模式
启用调试输出：
```bash
roslaunch fbrtkros fbrtk_driver.launch --screen
```

## 数据格式

### IMU 数据协议
帧格式：`0x55 [类型] [数据0-7] [校验和]`
- 0x51: 加速度数据 (±2g范围)
- 0x52: 角速度数据 (±250°/s范围)
- 0x53: 欧拉角数据 (±180°范围)

### GPS 数据协议
帧格式：`0x55 [类型] [数据0-7] [校验和]`
- 0x57: 经度/纬度 (度，7位小数精度)
- 0x58: 海拔/卫星数/Fix质量/速度(节)

### 控制协议
命令格式：`AT+CTRL=[角速度],[线速度]\r\n`
- 速度值按 32767 倍缩放
- 例如：`AT+CTRL=0,16384\r\n`

## 参与贡献

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request


