# #!/usr/bin/env python3
# """
# ROS2 node: IMU + Magnetometer LPF filter
# - accelerometer + gyro: moving average + LPF + gyro bias compensation
# - magnetometer: LPF
# - QoS: Best Effort
# - Output: /imu/filtered (sensor_msgs/Imu) + /mag/filtered (sensor_msgs/MagneticField)
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu, MagneticField
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# from collections import deque


# class ImuMagLpfFilter(Node):
#     def __init__(self):
#         super().__init__('imu_mag_lpf_filter')

#         # Parameters
#         self.declare_parameter('window_size', 5)
#         self.declare_parameter('alpha', 0.3)
#         self.declare_parameter('mag_alpha', 0.3)
#         self.declare_parameter('bias_alpha', 0.0003)  # gyro bias learning

#         self.window_size = self.get_parameter('window_size').value
#         self.alpha = self.get_parameter('alpha').value
#         self.mag_alpha = self.get_parameter('mag_alpha').value
#         self.bias_alpha = self.get_parameter('bias_alpha').value

#         # Buffers
#         self.acc_x_buf = deque(maxlen=self.window_size)
#         self.acc_y_buf = deque(maxlen=self.window_size)
#         self.acc_z_buf = deque(maxlen=self.window_size)

#         self.gyro_x_buf = deque(maxlen=self.window_size)
#         self.gyro_y_buf = deque(maxlen=self.window_size)
#         self.gyro_z_buf = deque(maxlen=self.window_size)

#         # Previous filtered values
#         self.prev_acc = [0.0, 0.0, 0.0]
#         self.prev_gyro = [0.0, 0.0, 0.0]
#         self.gyro_bias = [0.0, 0.0, 0.0]
#         self.prev_mag = [0.0, 0.0, 0.0]

#         # QoS Best Effort
#         qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

#         # Subscribers
#         self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
#         self.sub_mag = self.create_subscription(MagneticField, '/mag/data', self.mag_callback, qos)

#         # Publishers
#         self.pub_imu = self.create_publisher(Imu, '/imu/filtered', qos)
#         self.pub_mag = self.create_publisher(MagneticField, '/mag/filtered', qos)

#         self.get_logger().info(
#             f"IMU+Mag LPF Filter running (window={self.window_size}, alpha={self.alpha}, mag_alpha={self.mag_alpha}, QoS=BEST_EFFORT)"
#         )

#     # --- Moving Average ---
#     def mov_avg(self, buf, new_value):
#         buf.append(new_value)
#         return sum(buf) / len(buf)

#     # --- LPF ---
#     def lpf(self, prev, current, alpha):
#         return alpha * current + (1 - alpha) * prev

#     # --- Gyro bias update ---
#     def update_gyro_bias(self, idx, raw):
#         self.gyro_bias[idx] = (1 - self.bias_alpha) * self.gyro_bias[idx] + self.bias_alpha * raw
#         return raw - self.gyro_bias[idx]

#     # --- IMU callback ---
#     def imu_callback(self, msg: Imu):
#         out = Imu()
#         out.header = msg.header

#         # Accelerometer MA + LPF
#         ax_avg = self.mov_avg(self.acc_x_buf, msg.linear_acceleration.x)
#         ay_avg = self.mov_avg(self.acc_y_buf, msg.linear_acceleration.y)
#         az_avg = self.mov_avg(self.acc_z_buf, msg.linear_acceleration.z)

#         out.linear_acceleration.x = self.prev_acc[0] = self.lpf(self.prev_acc[0], ax_avg, self.alpha)
#         out.linear_acceleration.y = self.prev_acc[1] = self.lpf(self.prev_acc[1], ay_avg, self.alpha)
#         out.linear_acceleration.z = self.prev_acc[2] = self.lpf(self.prev_acc[2], az_avg, self.alpha)

#         # Gyro bias compensation + MA + LPF
#         gx_corr = self.update_gyro_bias(0, msg.angular_velocity.x)
#         gy_corr = self.update_gyro_bias(1, msg.angular_velocity.y)
#         gz_corr = self.update_gyro_bias(2, msg.angular_velocity.z)

#         gx_avg = self.mov_avg(self.gyro_x_buf, gx_corr)
#         gy_avg = self.mov_avg(self.gyro_y_buf, gy_corr)
#         gz_avg = self.mov_avg(self.gyro_z_buf, gz_corr)

#         out.angular_velocity.x = self.prev_gyro[0] = self.lpf(self.prev_gyro[0], gx_avg, self.alpha)
#         out.angular_velocity.y = self.prev_gyro[1] = self.lpf(self.prev_gyro[1], gy_avg, self.alpha)
#         out.angular_velocity.z = self.prev_gyro[2] = self.lpf(self.prev_gyro[2], gz_avg, self.alpha)

#         # Orientation giữ nguyên
#         out.orientation = msg.orientation

#         self.pub_imu.publish(out)

#     # --- Magnetometer callback ---
#     def mag_callback(self, msg: MagneticField):
#         out = MagneticField()
#         out.header = msg.header

#         out.magnetic_field.x = self.prev_mag[0] = self.lpf(self.prev_mag[0], msg.magnetic_field.x, self.mag_alpha)
#         out.magnetic_field.y = self.prev_mag[1] = self.lpf(self.prev_mag[1], msg.magnetic_field.y, self.mag_alpha)
#         out.magnetic_field.z = self.prev_mag[2] = self.lpf(self.prev_mag[2], msg.magnetic_field.z, self.mag_alpha)

#         self.pub_mag.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuMagLpfFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy
from collections import deque


class ImuMagFilter(Node):

    def __init__(self):
        super().__init__('imu_mag_optimized_filter')

        # Parameters
        self.declare_parameter('window_size', 3)
        self.declare_parameter('alpha', 0.25)
        self.declare_parameter('mag_alpha', 0.25)
        self.declare_parameter('bias_alpha', 0.0003)

        self.N = self.get_parameter('window_size').value
        self.alpha = self.get_parameter('alpha').value
        self.mag_alpha = self.get_parameter('mag_alpha').value
        self.bias_alpha = self.get_parameter('bias_alpha').value

        # Queues for moving average
        self.acc = [deque(maxlen=self.N), deque(maxlen=self.N), deque(maxlen=self.N)]
        self.gyro = [deque(maxlen=self.N), deque(maxlen=self.N), deque(maxlen=self.N)]

        # Previous filtered outputs
        self.acc_prev = [0.0, 0.0, 0.0]
        self.gyro_prev = [0.0, 0.0, 0.0]
        self.mag_prev = [0.0, 0.0, 0.0]

        # Gyro bias
        self.gyro_bias = [0.0, 0.0, 0.0]

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Sub
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
        self.create_subscription(MagneticField, '/mag/data', self.mag_callback, qos)

        # Pub
        self.pub_imu = self.create_publisher(Imu, '/imu/filtered', qos)
        self.pub_mag = self.create_publisher(MagneticField, '/mag/filtered', qos)

        self.get_logger().info("Optimized IMU + MAG filter running")

    # --- fast moving average ---
    def moving_avg(self, buf, value):
        buf.append(value)
        return sum(buf) / len(buf)

    # --- Low-pass filter ---
    def lpf(self, prev, current, a):
        return prev + a * (current - prev)

    # --- Gyro bias update ---
    def update_bias(self, idx, raw):
        b = self.gyro_bias[idx]
        b = b + self.bias_alpha * (raw - b)
        self.gyro_bias[idx] = b
        return raw - b

    # ========== IMU ==========
    def imu_callback(self, msg):
        out = Imu()
        out.header = msg.header

        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # --- ACC ---
        acc_values = [ax, ay, az]
        for i in range(3):
            avg = self.moving_avg(self.acc[i], acc_values[i])
            self.acc_prev[i] = self.lpf(self.acc_prev[i], avg, self.alpha)

        out.linear_acceleration.x = self.acc_prev[0]
        out.linear_acceleration.y = self.acc_prev[1]
        out.linear_acceleration.z = self.acc_prev[2]

        # --- GYRO ---
        gyro_raw = [gx, gy, gz]
        for i in range(3):
            corrected = self.update_bias(i, gyro_raw[i])  # bias remove
            avg = self.moving_avg(self.gyro[i], corrected)
            self.gyro_prev[i] = self.lpf(self.gyro_prev[i], avg, self.alpha)

        out.angular_velocity.x = self.gyro_prev[0]
        out.angular_velocity.y = self.gyro_prev[1]
        out.angular_velocity.z = self.gyro_prev[2]

        # giữ orientation nguyên
        out.orientation = msg.orientation

        self.pub_imu.publish(out)

    # ========== MAG ==========
    def mag_callback(self, msg):
        out = MagneticField()
        out.header = msg.header

        mx, my, mz = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z

        self.mag_prev[0] = self.lpf(self.mag_prev[0],  mx, self.mag_alpha)
        self.mag_prev[1] = self.lpf(self.mag_prev[1], -my, self.mag_alpha)
        self.mag_prev[2] = self.lpf(self.mag_prev[2], -mz, self.mag_alpha)

        out.magnetic_field.x = self.mag_prev[0]
        out.magnetic_field.y = self.mag_prev[1]
        out.magnetic_field.z = self.mag_prev[2]

        self.pub_mag.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMagFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# """
# Optimized ROS2 node: IMU + Magnetometer LPF filter (improvements on receiving phase)
# Features:
#  - accelerometer + gyro: moving average (small window) + LPF + gyro bias compensation
#  - magnetometer: LPF + optional median + scaling to Tesla
#  - optional time-based LPF using message header timestamps
#  - QoS: Best Effort
#  - Publishes: /imu/filtered (sensor_msgs/Imu) and /mag/filtered (sensor_msgs/MagneticField)
# """
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu, MagneticField
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# from collections import deque
# import math
# import statistics

# class ImuMagLpfFilter(Node):
#     def __init__(self):
#         super().__init__('imu_mag_lpf_filter')

#         # --- Parameters (tunable) ---
#         # moving average size (small to reduce delay)
#         self.declare_parameter('window_size', 3)

#         # fallback fixed alpha (if not using time-based)
#         self.declare_parameter('alpha', 0.35)
#         self.declare_parameter('mag_alpha', 0.2)

#         # or use time-based LPF: alpha_dt = dt / (rc + dt)
#         self.declare_parameter('use_time_based', True)
#         self.declare_parameter('rc_acc', 0.05)   # seconds
#         self.declare_parameter('rc_gyro', 0.05)
#         self.declare_parameter('rc_mag', 0.1)

#         # gyro bias adaptation (increase to learn faster at startup)
#         self.declare_parameter('bias_alpha', 0.002)

#         # max dt: skip extremely delayed samples
#         self.declare_parameter('max_dt', 0.2)

#         # magnetometer scaling to Tesla (default assume input in microTesla; µT -> T = *1e-6)
#         # For AK09916 set mag_scale=0.15e-6
#         self.declare_parameter('mag_scale', 1e-6)
#         # outlier rejection for mag (absolute units before scaling)
#         self.declare_parameter('mag_outlier_limit', 1000.0)  # large default; tune as needed

#         # optionally use median filter for mag to remove spikes
#         self.declare_parameter('use_median_for_mag', False)
#         self.declare_parameter('median_window', 3)

#         # read params
#         self.window_size = int(self.get_parameter('window_size').value)
#         self.alpha = float(self.get_parameter('alpha').value)
#         self.mag_alpha = float(self.get_parameter('mag_alpha').value)
#         self.use_time_based = bool(self.get_parameter('use_time_based').value)
#         self.rc_acc = float(self.get_parameter('rc_acc').value)
#         self.rc_gyro = float(self.get_parameter('rc_gyro').value)
#         self.rc_mag = float(self.get_parameter('rc_mag').value)
#         self.bias_alpha = float(self.get_parameter('bias_alpha').value)
#         self.max_dt = float(self.get_parameter('max_dt').value)
#         self.mag_scale = float(self.get_parameter('mag_scale').value)
#         self.mag_outlier_limit = float(self.get_parameter('mag_outlier_limit').value)
#         self.use_median_for_mag = bool(self.get_parameter('use_median_for_mag').value)
#         self.median_window = int(self.get_parameter('median_window').value)
#         if self.median_window < 3:
#             self.median_window = 3

#         # Buffers for moving average
#         self.acc_x_buf = deque(maxlen=self.window_size)
#         self.acc_y_buf = deque(maxlen=self.window_size)
#         self.acc_z_buf = deque(maxlen=self.window_size)
#         self.gyro_x_buf = deque(maxlen=self.window_size)
#         self.gyro_y_buf = deque(maxlen=self.window_size)
#         self.gyro_z_buf = deque(maxlen=self.window_size)

#         # prev filtered values (state)
#         self.prev_acc = [0.0, 0.0, 0.0]
#         self.prev_gyro = [0.0, 0.0, 0.0]
#         self.gyro_bias = [0.0, 0.0, 0.0]
#         self.prev_mag = [0.0, 0.0, 0.0]

#         # median buffers for mag if used
#         if self.use_median_for_mag:
#             self.mag_x_buf = deque(maxlen=self.median_window)
#             self.mag_y_buf = deque(maxlen=self.median_window)
#             self.mag_z_buf = deque(maxlen=self.median_window)

#         # time state
#         self.last_time = None

#         # QoS Best Effort
#         qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

#         # Subscriptions & publishers
#         self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
#         self.sub_mag = self.create_subscription(MagneticField, '/mag/data', self.mag_callback, qos)
#         self.pub_imu = self.create_publisher(Imu, '/imu/filtered', qos)
#         self.pub_mag = self.create_publisher(MagneticField, '/mag/filtered', qos)

#         self.get_logger().info(
#             f"IMU+Mag LPF running (win={self.window_size}, alpha={self.alpha}, mag_alpha={self.mag_alpha}, "
#             f"use_time_based={self.use_time_based}, mag_scale={self.mag_scale})"
#         )

#     # -------------------------
#     # Helpers
#     # -------------------------
#     def mov_avg(self, buf, new_value):
#         buf.append(new_value)
#         return sum(buf) / len(buf)

#     def median_if_enabled(self, buf, new_value):
#         buf.append(new_value)
#         if len(buf) >= buf.maxlen:
#             return float(statistics.median(buf))
#         else:
#             return float(sum(buf) / len(buf))

#     def lpf_fixed(self, prev, current, alpha):
#         return alpha * current + (1 - alpha) * prev

#     def lpf_time_based(self, prev, current, dt, rc):
#         if dt <= 0:
#             return prev
#         alpha_dt = dt / (rc + dt)
#         return prev + alpha_dt * (current - prev)

#     def _get_msg_time(self, msg):
#         # prefer header stamp if valid (>0), else node clock
#         try:
#             sec = msg.header.stamp.sec
#             nsec = msg.header.stamp.nanosec
#             if sec == 0 and nsec == 0:
#                 raise ValueError("zero stamp")
#             return float(sec) + float(nsec) * 1e-9
#         except Exception:
#             return self.get_clock().now().nanoseconds * 1e-9

#     # -------------------------
#     # Callbacks
#     # -------------------------
#     def imu_callback(self, msg: Imu):
#         now = self._get_msg_time(msg)
#         if self.last_time is None:
#             # seed state and publish initial
#             self.last_time = now
#             # seed buffers
#             self.acc_x_buf.append(msg.linear_acceleration.x)
#             self.acc_y_buf.append(msg.linear_acceleration.y)
#             self.acc_z_buf.append(msg.linear_acceleration.z)
#             self.gyro_x_buf.append(msg.angular_velocity.x)
#             self.gyro_y_buf.append(msg.angular_velocity.y)
#             self.gyro_z_buf.append(msg.angular_velocity.z)
#             # initial prev = raw
#             self.prev_acc = [
#                 msg.linear_acceleration.x,
#                 msg.linear_acceleration.y,
#                 msg.linear_acceleration.z
#             ]
#             self.prev_gyro = [
#                 msg.angular_velocity.x,
#                 msg.angular_velocity.y,
#                 msg.angular_velocity.z
#             ]
#             out0 = Imu()
#             out0.header = msg.header
#             out0.linear_acceleration.x = self.prev_acc[0]
#             out0.linear_acceleration.y = self.prev_acc[1]
#             out0.linear_acceleration.z = self.prev_acc[2]
#             out0.angular_velocity.x = self.prev_gyro[0]
#             out0.angular_velocity.y = self.prev_gyro[1]
#             out0.angular_velocity.z = self.prev_gyro[2]
#             # covariances (inform EKF)
#             out0.linear_acceleration_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]
#             out0.angular_velocity_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]
#             out0.orientation_covariance = [-1.0,0,0, 0,-1.0,0, 0,0,-1.0]
#             out0.orientation = msg.orientation
#             self.pub_imu.publish(out0)
#             return

#         dt = now - self.last_time
#         self.last_time = now

#         # skip if dt huge
#         if dt > self.max_dt:
#             self.get_logger().debug(f"Skipping IMU sample dt={dt:.3f}s > max_dt={self.max_dt}")
#             return

#         # --- ACC: mov avg (small window) + LPF ---
#         ax_avg = self.mov_avg(self.acc_x_buf, msg.linear_acceleration.x)
#         ay_avg = self.mov_avg(self.acc_y_buf, msg.linear_acceleration.y)
#         az_avg = self.mov_avg(self.acc_z_buf, msg.linear_acceleration.z)

#         if self.use_time_based:
#             fx = self.lpf_time_based(self.prev_acc[0], ax_avg, dt, self.rc_acc)
#             fy = self.lpf_time_based(self.prev_acc[1], ay_avg, dt, self.rc_acc)
#             fz = self.lpf_time_based(self.prev_acc[2], az_avg, dt, self.rc_acc)
#         else:
#             fx = self.lpf_fixed(self.prev_acc[0], ax_avg, self.alpha)
#             fy = self.lpf_fixed(self.prev_acc[1], ay_avg, self.alpha)
#             fz = self.lpf_fixed(self.prev_acc[2], az_avg, self.alpha)

#         self.prev_acc[0], self.prev_acc[1], self.prev_acc[2] = fx, fy, fz

#         # --- GYRO: bias compensation -> mov avg -> LPF (time-based if set) ---
#         gx_corr = (1.0 - self.bias_alpha) * self.gyro_bias[0] + self.bias_alpha * msg.angular_velocity.x
#         gy_corr = (1.0 - self.bias_alpha) * self.gyro_bias[1] + self.bias_alpha * msg.angular_velocity.y
#         gz_corr = (1.0 - self.bias_alpha) * self.gyro_bias[2] + self.bias_alpha * msg.angular_velocity.z

#         # update stored bias and subtract
#         self.gyro_bias[0] = gx_corr
#         self.gyro_bias[1] = gy_corr
#         self.gyro_bias[2] = gz_corr

#         gx = msg.angular_velocity.x - self.gyro_bias[0]
#         gy = msg.angular_velocity.y - self.gyro_bias[1]
#         gz = msg.angular_velocity.z - self.gyro_bias[2]

#         gx_avg = self.mov_avg(self.gyro_x_buf, gx)
#         gy_avg = self.mov_avg(self.gyro_y_buf, gy)
#         gz_avg = self.mov_avg(self.gyro_z_buf, gz)

#         if self.use_time_based:
#             fgx = self.lpf_time_based(self.prev_gyro[0], gx_avg, dt, self.rc_gyro)
#             fgy = self.lpf_time_based(self.prev_gyro[1], gy_avg, dt, self.rc_gyro)
#             fgz = self.lpf_time_based(self.prev_gyro[2], gz_avg, dt, self.rc_gyro)
#         else:
#             fgx = self.lpf_fixed(self.prev_gyro[0], gx_avg, self.alpha)
#             fgy = self.lpf_fixed(self.prev_gyro[1], gy_avg, self.alpha)
#             fgz = self.lpf_fixed(self.prev_gyro[2], gz_avg, self.alpha)

#         self.prev_gyro[0], self.prev_gyro[1], self.prev_gyro[2] = fgx, fgy, fgz

#         # --- Prepare IMU output ---
#         out = Imu()
#         out.header = msg.header
#         out.linear_acceleration.x = fx
#         out.linear_acceleration.y = fy
#         out.linear_acceleration.z = fz
#         out.angular_velocity.x = fgx
#         out.angular_velocity.y = fgy
#         out.angular_velocity.z = fgz
#         out.orientation = msg.orientation  # keep raw orientation as requested

#         # Covariances for EKF
#         out.linear_acceleration_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]
#         out.angular_velocity_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]
#         out.orientation_covariance = [-1.0,0,0, 0,-1.0,0, 0,0,-1.0]

#         self.pub_imu.publish(out)

#     def mag_callback(self, msg: MagneticField):
#         # scale raw mag -> Tesla (sensor dependent). Default assumes µT input.
#         # If your sensor raw LSB -> use appropriate mag_scale (e.g. AK09916 -> 0.15e-6)
#         mx = msg.magnetic_field.x * self.mag_scale
#         my = msg.magnetic_field.y * self.mag_scale
#         mz = msg.magnetic_field.z * self.mag_scale

#         # outlier reject before filtering (in scaled units)
#         if (abs(mx) > self.mag_outlier_limit) or (abs(my) > self.mag_outlier_limit) or (abs(mz) > self.mag_outlier_limit):
#             self.get_logger().debug("Mag outlier rejected")
#             return

#         # optional median to cut spikes
#         if self.use_median_for_mag:
#             self.mag_x_buf.append(mx)
#             self.mag_y_buf.append(my)
#             self.mag_z_buf.append(mz)
#             mx_clean = statistics.median(self.mag_x_buf) if len(self.mag_x_buf) >= self.mag_x_buf.maxlen else sum(self.mag_x_buf)/len(self.mag_x_buf)
#             my_clean = statistics.median(self.mag_y_buf) if len(self.mag_y_buf) >= self.mag_y_buf.maxlen else sum(self.mag_y_buf)/len(self.mag_y_buf)
#             mz_clean = statistics.median(self.mag_z_buf) if len(self.mag_z_buf) >= self.mag_z_buf.maxlen else sum(self.mag_z_buf)/len(self.mag_z_buf)
#         else:
#             mx_clean, my_clean, mz_clean = mx, my, mz

#         # time-based LPF for mag uses last_time from imu; if None, treat as fixed alpha
#         # we try to use node clock if imu hasn't set last_time yet
#         now = self.get_clock().now().nanoseconds * 1e-9
#         dt = 0.0
#         if self.last_time is not None:
#             dt = now - self.last_time
#             if dt <= 0 or dt > self.max_dt:
#                 # fallback to fixed LPF alpha
#                 alpha_mag = self.mag_alpha
#                 fx = self.lpf_fixed(self.prev_mag[0], mx_clean, alpha_mag)
#                 fy = self.lpf_fixed(self.prev_mag[1], my_clean, alpha_mag)
#                 fz = self.lpf_fixed(self.prev_mag[2], mz_clean, alpha_mag)
#             else:
#                 fx = self.lpf_time_based(self.prev_mag[0], mx_clean, dt, self.rc_mag)
#                 fy = self.lpf_time_based(self.prev_mag[1], my_clean, dt, self.rc_mag)
#                 fz = self.lpf_time_based(self.prev_mag[2], mz_clean, dt, self.rc_mag)
#         else:
#             fx = self.lpf_fixed(self.prev_mag[0], mx_clean, self.mag_alpha)
#             fy = self.lpf_fixed(self.prev_mag[1], my_clean, self.mag_alpha)
#             fz = self.lpf_fixed(self.prev_mag[2], mz_clean, self.mag_alpha)

#         self.prev_mag[0], self.prev_mag[1], self.prev_mag[2] = fx, fy, fz

#         out = MagneticField()
#         out.header.stamp = self.get_clock().now().to_msg()
#         out.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'imu_link'
#         out.magnetic_field.x = fx
#         out.magnetic_field.y = fy
#         out.magnetic_field.z = fz

#         # covariance for EKF
#         out.magnetic_field_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]

#         self.pub_mag.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuMagLpfFilter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


