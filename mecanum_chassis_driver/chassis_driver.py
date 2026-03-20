#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import serial
import threading
import math
import tf2_ros
from typing import Tuple, List
import re


class MecanumChassisDriver(Node):
    """
    四驱麦轮底盘驱动节点
    
    坐标系定义（ROS2标准）：
    - X轴：向前为正
    - Y轴：向左为正  
    - Z轴：向上为正
    - 旋转：逆时针为正（从上方看）
    
    电机布局：
    - M1: 左前轮 (Left-Front)
    - M2: 左后轮 (Left-Rear)
    - M3: 右前轮 (Right-Front)
    - M4: 右后轮 (Right-Rear)
    
    逆运动学（给定底盘速度，计算各轮速度）：
    - v1(M1) = Vx - Vy - (L+W)*Omega
    - v2(M2) = Vx + Vy - (L+W)*Omega
    - v3(M3) = Vx + Vy + (L+W)*Omega
    - v4(M4) = Vx - Vy + (L+W)*Omega
    
    正运动学（给定各轮位移，计算底盘位移）：
    - dx = (d1 + d2 + d3 + d4) / 4
    - dy = (-d1 + d2 - d3 + d4) / 4
    - dtheta = (-d1 - d2 + d3 + d4) / (4*(L+W))
    """

    def __init__(self):
        super().__init__('mecanum_chassis_driver')

        # 参数声明
        self.declare_parameters(namespace='', parameters=[
            ('port', '/dev/ttyUSB0'),
            ('baudrate', 115200),
            ('wheel_diameter', 0.067),      # 67mm，根据文档默认67
            ('wheel_base', 0.15),            # 半轮距（左右轮中心距的一半）
            ('axle_base', 0.15),             # 半轴距（前后轮中心距的一半）
            ('encoder_resolution', 1560),    # 编码器线数*减速比
            ('control_rate', 50.0),          # 控制频率Hz
            ('odom_rate', 50.0),             # 里程计发布频率
            ('speed_scale', 1000.0),         # 速度指令最大值对应实际速度(m/s)
            ('upload_encoder', True),        # 是否开启编码器数据上报
            ('encoder_max', 2147483647),     # 编码器最大值（用于溢出检测）
            ('encoder_min', -2147483648),    # 编码器最小值
        ])

        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_d = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.axle_base = self.get_parameter('axle_base').value
        self.encoder_res = self.get_parameter('encoder_resolution').value
        self.speed_scale = self.get_parameter('speed_scale').value
        self.encoder_max = self.get_parameter('encoder_max').value
        self.encoder_min = self.get_parameter('encoder_min').value

        # 麦轮运动学参数
        self.l_plus_w = self.wheel_base + self.axle_base

        # 线程锁（保护共享数据）
        self.encoder_lock = threading.Lock()
        self.speeds_lock = threading.Lock()

        # 串口初始化
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'串口已打开: {self.port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            raise

        # 状态变量（受锁保护）
        self.current_twist = Twist()
        with self.speeds_lock:
            self.target_speeds = [0, 0, 0, 0]  # M1, M2, M3, M4
        with self.encoder_lock:
            self.encoder_data = [0, 0, 0, 0]           # 当前编码器值
            self.last_encoder_data = [0, 0, 0, 0]      # 上次编码器值
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS接口
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.raw_speed_pub = self.create_publisher(
            Float32MultiArray, 'wheel_speeds', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定时器
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_rate').value,
            self.control_loop)
        self.odom_timer = self.create_timer(
            1.0 / self.get_parameter('odom_rate').value,
            self.odom_loop)

        # 接收线程
        self.recv_thread = threading.Thread(target=self.serial_receive_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        # 初始化电机驱动板
        self.init_motor_driver()

    def init_motor_driver(self):
        """初始化电机驱动板配置"""
        # 配置轮子直径（单位mm）
        diameter_mm = int(self.wheel_d * 1000)
        cmd = f'$wdiameter:{diameter_mm}#\n'
        try:
            self.ser.write(cmd.encode())
            self.get_logger().info(f'发送轮子直径配置: {diameter_mm}mm')
        except Exception as e:
            self.get_logger().error(f'发送轮子直径配置失败: {e}')

        # 开启编码器数据上报（如果需要）
        if self.get_parameter('upload_encoder').value:
            # $upload:总编码器,实时编码器,速度(1=开,0=关)
            cmd = '$upload:1,1,1#\n'
            try:
                self.ser.write(cmd.encode())
                self.get_logger().info('开启编码器数据自动上报')
            except Exception as e:
                self.get_logger().error(f'开启编码器上报失败: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """接收速度指令，解算为四轮速度"""
        self.current_twist = msg
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 麦轮逆运动学
        # M1:左前, M2:左后, M3:右前, M4:右后
        v_m1 = vx - vy - self.l_plus_w * wz
        v_m2 = vx + vy - self.l_plus_w * wz
        v_m3 = vx + vy + self.l_plus_w * wz
        v_m4 = vx - vy + self.l_plus_w * wz

        # 转换为电机驱动板速度值（-1000 ~ 1000）
        max_lin_speed = self.speed_scale

        speeds = [
            int((v_m1 / max_lin_speed) * 1000),
            int((v_m2 / max_lin_speed) * 1000),
            int((v_m3 / max_lin_speed) * 1000),
            int((v_m4 / max_lin_speed) * 1000)
        ]

        # 限幅
        for i in range(4):
            speeds[i] = max(-1000, min(1000, speeds[i]))

        with self.speeds_lock:
            self.target_speeds = speeds

    def control_loop(self):
        """定时发送速度控制指令"""
        with self.speeds_lock:
            speeds = self.target_speeds.copy()
        cmd = f'$spd:{speeds[0]},{speeds[1]},{speeds[2]},{speeds[3]}#\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'串口写入失败: {e}')

    def serial_receive_loop(self):
        """后台线程接收编码器数据"""
        buffer = ''
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode(
                        'utf-8', errors='ignore')
                    buffer += data

                    # 处理完整数据包（以#结尾）
                    while '#' in buffer:
                        idx = buffer.find('#')
                        line = buffer[:idx]
                        buffer = buffer[idx + 1:]

                        if line.startswith('$MALL'):  # 总编码器数据
                            self.parse_encoder_data(line, 'all')
                        elif line.startswith('$MTEP'):  # 实时编码器数据
                            self.parse_encoder_data(line, 'instant')
                        elif line.startswith('$MSPD'):  # 轮子速度
                            self.parse_wheel_speed(line)
                        elif line.startswith('$Battery'):  # 电池电量
                            self.parse_battery(line)
                        elif 'OK' in line:
                            self.get_logger().debug(f'指令确认: {line}')

            except Exception as e:
                self.get_logger().error(f'串口接收错误: {e}')

    def parse_encoder_data(self, data: str, data_type: str):
        """解析编码器数据 $MALL:M1,M2,M3,M4#"""
        try:
            match = re.match(r'\$MALL:(-?\d+),(-?\d+),(-?\d+),(-?\d+)', data)
            if match:
                encoders = [int(match.group(i)) for i in range(1, 5)]
                if data_type == 'all':
                    with self.encoder_lock:
                        self.encoder_data = encoders
            else:
                self.get_logger().warn(f'编码器数据格式不匹配: {data}')
        except Exception as e:
            self.get_logger().warn(f'解析编码器数据失败: {data}, 错误: {e}')

    def parse_wheel_speed(self, data: str):
        """解析轮子速度反馈 $MSPD:M1,M2,M3,M4#"""
        try:
            match = re.match(r'\$MSPD:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)', data)
            if match:
                speeds = [float(match.group(i)) for i in range(1, 5)]
                msg = Float32MultiArray()
                msg.data = speeds
                self.raw_speed_pub.publish(msg)
            else:
                self.get_logger().debug(f'轮子速度数据格式不匹配: {data}')
        except Exception as e:
            self.get_logger().debug(f'解析轮子速度失败: {data}, 错误: {e}')

    def parse_battery(self, data: str):
        """解析电池电量 $Battery:7.40V#"""
        try:
            match = re.search(r'(\d+\.\d+)', data)
            if match:
                voltage = float(match.group(1))
                self.get_logger().info(f'电池电压: {voltage}V', throttle_duration_sec=10.0)
            else:
                self.get_logger().debug(f'电池数据格式不匹配: {data}')
        except Exception as e:
            self.get_logger().debug(f'解析电池数据失败: {data}, 错误: {e}')

    def handle_encoder_overflow(self, current: int, last: int) -> int:
        """处理编码器溢出/回绕"""
        diff = current - last
        # 检测正向溢出
        if diff < -(self.encoder_max - self.encoder_min) // 2:
            diff += (self.encoder_max - self.encoder_min + 1)
        # 检测反向溢出
        elif diff > (self.encoder_max - self.encoder_min) // 2:
            diff -= (self.encoder_max - self.encoder_min + 1)
        return diff

    def calculate_odometry(self):
        """根据编码器数据计算里程计"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        with self.encoder_lock:
            current_encoders = self.encoder_data.copy()
            last_encoders = self.last_encoder_data.copy()
            # 立即更新 last_encoder_data，避免数据竞争
            self.last_encoder_data = current_encoders.copy()

        # 计算编码器差值（处理溢出）
        delta_enc = [
            self.handle_encoder_overflow(current_encoders[i], last_encoders[i])
            for i in range(4)
        ]

        # 转换为轮子线位移（米）
        wheel_circumference = math.pi * self.wheel_d
        delta_dist = [
            (delta_enc[i] / self.encoder_res) * wheel_circumference
            for i in range(4)
        ]

        # 麦轮正运动学（从四轮位移计算底盘位移）
        d_m1, d_m2, d_m3, d_m4 = delta_dist

        # 底盘坐标系下的位移
        # 推导过程：
        # 逆运动学矩阵 A = [[1, -1, -L], [1, 1, -L], [1, 1, L], [1, -1, L]]
        # 其中列分别对应 Vx, Vy, Omega 的系数
        # 正运动学需要计算 (A^T * A)^-1 * A^T * d
        # 计算得：
        # dx = (d1 + d2 + d3 + d4) / 4
        # dy = (-d1 + d2 - d3 + d4) / 4
        # dtheta = (-d1 - d2 + d3 + d4) / (4*(L+W))
        dx = (d_m1 + d_m2 + d_m3 + d_m4) / 4.0
        dy = (-d_m1 + d_m2 - d_m3 + d_m4) / 4.0
        dtheta = (-d_m1 - d_m2 + d_m3 + d_m4) / (4.0 * self.l_plus_w)

        # 更新位姿（在世界坐标系下）
        self.theta += dtheta
        # 归一化角度到 [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)

        # 计算速度
        vx = dx / dt
        vy = dy / dt
        vth = dtheta / dt

        # 发布Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 四元数转换
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # 协方差（可根据需要调整）
        odom.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 0.01, 0, 0, 0,
                                0, 0, 0, 0.01, 0, 0,
                                0, 0, 0, 0, 0.01, 0,
                                0, 0, 0, 0, 0, 0.01]
        odom.twist.covariance = odom.pose.covariance.copy()

        self.odom_pub.publish(odom)

        # 发布TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 更新时间
        self.last_time = current_time

    def odom_loop(self):
        """里程计计算循环"""
        self.calculate_odometry()

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在关闭节点，停止电机...')
        # 停止电机
        try:
            self.ser.write('$spd:0,0,0,0#\n'.encode())
            self.ser.close()
            self.get_logger().info('串口已关闭')
        except Exception as e:
            self.get_logger().warn(f'关闭串口时出错: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumChassisDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
