#!/usr/bin/env python3
"""
ROS2 node: IMU + Magnetometer LPF filter
- accelerometer + gyro: moving average + LPF + gyro bias compensation
- magnetometer: LPF
- QoS: Best Effort
- Output: /imu/filtered (sensor_msgs/Imu) + /mag/filtered (sensor_msgs/MagneticField)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy
from collections import deque

class ImuMagLpfFilter(Node):
    def __init__(self):
        super().__init__('imu_mag_lpf_filter')

        # Parameters
        self.declare_parameter('window_size', 5)
        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('mag_alpha', 0.3)
        self.declare_parameter('bias_alpha', 0.0003)  # gyro bias learning

        self.window_size = self.get_parameter('window_size').value
        self.alpha = self.get_parameter('alpha').value
        self.mag_alpha = self.get_parameter('mag_alpha').value
        self.bias_alpha = self.get_parameter('bias_alpha').value

        # Buffers
        self.acc_x_buf = deque(maxlen=self.window_size)
        self.acc_y_buf = deque(maxlen=self.window_size)
        self.acc_z_buf = deque(maxlen=self.window_size)

        self.gyro_x_buf = deque(maxlen=self.window_size)
        self.gyro_y_buf = deque(maxlen=self.window_size)
        self.gyro_z_buf = deque(maxlen=self.window_size)

        # Previous filtered values
        self.prev_acc = [0.0, 0.0, 0.0]
        self.prev_gyro = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]

        self.prev_mag = [0.0, 0.0, 0.0]

        # QoS Best Effort
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
        self.sub_mag = self.create_subscription(MagneticField, '/mag/data', self.mag_callback, qos)

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/filtered', qos)
        self.pub_mag = self.create_publisher(MagneticField, '/mag/filtered', qos)

        self.get_logger().info(f"IMU+Mag LPF Filter running (window={self.window_size}, alpha={self.alpha}, mag_alpha={self.mag_alpha}, QoS=BEST_EFFORT)")

    # --- Moving Average ---
    def mov_avg(self, buf, new_value):
        buf.append(new_value)
        return sum(buf) / len(buf)

    # --- LPF ---
    def lpf(self, prev, current, alpha):
        return alpha * current + (1 - alpha) * prev

    # --- Gyro bias update ---
    def update_gyro_bias(self, idx, raw):
        self.gyro_bias[idx] = (1 - self.bias_alpha) * self.gyro_bias[idx] + self.bias_alpha * raw
        return raw - self.gyro_bias[idx]

    # --- IMU callback ---
    def imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # Accelerometer MA + LPF
        ax_avg = self.mov_avg(self.acc_x_buf, msg.linear_acceleration.x)
        ay_avg = self.mov_avg(self.acc_y_buf, msg.linear_acceleration.y)
        az_avg = self.mov_avg(self.acc_z_buf, msg.linear_acceleration.z)

        out.linear_acceleration.x = self.prev_acc[0] = self.lpf(self.prev_acc[0], ax_avg, self.alpha)
        out.linear_acceleration.y = self.prev_acc[1] = self.lpf(self.prev_acc[1], ay_avg, self.alpha)
        out.linear_acceleration.z = self.prev_acc[2] = self.lpf(self.prev_acc[2], az_avg, self.alpha)

        # Gyro bias compensation + MA + LPF
        gx_corr = self.update_gyro_bias(0, msg.angular_velocity.x)
        gy_corr = self.update_gyro_bias(1, msg.angular_velocity.y)
        gz_corr = self.update_gyro_bias(2, msg.angular_velocity.z)

        gx_avg = self.mov_avg(self.gyro_x_buf, gx_corr)
        gy_avg = self.mov_avg(self.gyro_y_buf, gy_corr)
        gz_avg = self.mov_avg(self.gyro_z_buf, gz_corr)

        out.angular_velocity.x = self.prev_gyro[0] = self.lpf(self.prev_gyro[0], gx_avg, self.alpha)
        out.angular_velocity.y = self.prev_gyro[1] = self.lpf(self.prev_gyro[1], gy_avg, self.alpha)
        out.angular_velocity.z = self.prev_gyro[2] = self.lpf(self.prev_gyro[2], gz_avg, self.alpha)

        # Orientation giữ nguyên
        out.orientation = msg.orientation

        self.pub_imu.publish(out)

    # --- Magnetometer callback ---
    def mag_callback(self, msg: MagneticField):
        out = MagneticField()
        out.header = msg.header

        out.magnetic_field.x = self.prev_mag[0] = self.lpf(self.prev_mag[0], msg.magnetic_field.x, self.mag_alpha)
        out.magnetic_field.y = self.prev_mag[1] = self.lpf(self.prev_mag[1], msg.magnetic_field.y, self.mag_alpha)
        out.magnetic_field.z = self.prev_mag[2] = self.lpf(self.prev_mag[2], msg.magnetic_field.z, self.mag_alpha)

        self.pub_mag.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuMagLpfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
