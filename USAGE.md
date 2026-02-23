# FB-RTK 驱动使用指南

## 快速开始

### 1. 启动驱动
```bash
# 启动 FB-RTK 驱动（包含传感器数据和机器人控制）
roslaunch fbrtkros fbrtk_driver.launch

# 或者带可视化启动
roslaunch fbrtkros fbrtk_visualization.launch
```

### 2. 测试数据接收
```bash
# 监控所有传感器数据
python3 /home/chenmingjie/catkin_ws/src/FB-RTK/fb-rtkros/scripts/monitor_data.py
```

### 3. 控制小车
```bash
# 使用键盘控制小车（5Hz发送/cmd_vel命令）
python3 /home/chenmingjie/catkin_ws/src/FB-RTK/fb-rtkros/scripts/simple_control.py
```

## 控制命令

### 键盘控制（推荐）
使用simple_control.py脚本进行实时键盘控制：
- **w**: 前进 
- **a**: 左转
- **d**: 右转
- **s**: 停止
- **q**: 退出

该脚本会以5Hz频率持续发送/cmd_vel命令，确保控制响应性。

### 直接发送速度命令
发送速度命令到 `/cmd_vel` 话题：
```bash
# 前进 0.5 m/s
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 原地左转
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# 停止
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## 监控数据

### 查看传感器话题
```bash
# IMU 数据（200Hz）
rostopic echo /fbrtk/imu/data

# GPS 位置信息
rostopic echo /fbrtk/gps/fix

# GPS 速度信息
rostopic echo /fbrtk/velocity

# 电池电压
rostopic echo /fbrtk/battery_voltage

# 气压（Pa）
rostopic echo /fbrtk/baro/pressure

# 气压计海拔（m）
rostopic echo /fbrtk/baro/altitude
```

### 查看话题频率
```bash
# 检查 IMU 数据频率（应该约为 200Hz）
rostopic hz /fbrtk/imu/data

# 检查 GPS 数据频率
rostopic hz /fbrtk/gps/fix
```

## 数据协议说明

### 传感器数据协议
- **帧格式**: `0x55 [类型] [数据0-7] [校验和]`
- **0x51**: 加速度数据
- **0x52**: 角速度数据  
- **0x53**: 欧拉角数据
- **0x54**: 原始角速度数据（忽略）
- **0x55**: 电池电压
- **0x57**: GPS 经纬度
- **0x58**: GPS 海拔、状态、速度（节）

### 控制命令协议
控制命令发送到设备使用AT指令格式：
- **格式**: `AT+CTRL=[角速度],[线速度]\r\n`
- 速度值按 32767 倍缩放后发送
- 例如：`AT+CTRL=0,16384\r\n`

## 故障排除

### 1. 串口权限问题
```bash
sudo chmod 666 /dev/ttyACM2
```

### 2. 无传感器数据
- 检查 `/dev/ttyACM2` 是否存在
- 确认设备波特率为 115200
- 检查设备电源和连接

### 3. 控制命令不响应
- 检查串口连接是否正常
- 确认设备是否支持AT+CTRL命令
- 检查速度值是否在合理范围内

### 4. IMU 数据异常
- IMU 数据应该以约 200Hz 频率输出
- 如果频率异常，检查串口通信是否稳定

## 参数配置

在启动文件中可以调整以下参数：
```xml
<!-- 串口配置（固定为 FB-RTK 设备） -->
<param name="port" value="/dev/ttyACM2" />
<param name="baudrate" value="115200" />

<!-- 速度限制 -->
<param name="max_linear_velocity" value="1.0" />  <!-- 最大线速度 m/s -->
<param name="max_angular_velocity" value="1.0" /> <!-- 最大角速度 rad/s -->

<!-- 气压计参数 -->
<param name="baro_frame_id" value="baro_link" />
<param name="baro_pressure_scale" value="1.0" />
<param name="baro_altitude_scale" value="0.01" />
```

## 发布的话题

### 传感器数据
- `/fbrtk/imu/data` (sensor_msgs/Imu): IMU数据，约200Hz
- `/fbrtk/gps/fix` (sensor_msgs/NavSatFix): GPS位置信息
- `/fbrtk/velocity` (geometry_msgs/TwistStamped): GPS速度信息
- `/fbrtk/battery_voltage` (std_msgs/Float32): 电池电压
- `/fbrtk/baro/pressure` (sensor_msgs/FluidPressure): 气压（Pa）
- `/fbrtk/baro/altitude` (std_msgs/Float32): 气压计海拔（m）

### 订阅的话题
- `/cmd_vel` (geometry_msgs/Twist): 机器人速度控制命令
