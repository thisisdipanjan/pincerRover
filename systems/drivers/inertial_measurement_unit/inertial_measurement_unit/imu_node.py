import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import threading
import time
import math

MPU6886_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0  
GYRO_SCALE = 131.0     
G = 9.80665


class MPU6886Node(Node):

    def __init__(self):
        super().__init__("mpu6886_node")

        self.publisher_ = self.create_publisher(Imu, "/imu/raw", 10)

        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU6886_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        self.lock = threading.Lock()
        self.running = True

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self.reader_thread = threading.Thread(
            target=self._imu_read_loop, daemon=True
        )
        self.reader_thread.start()

        self.create_timer(0.02, self.publish_imu)  # 50 Hz

        self.get_logger().info("MPU6886 IMU node started")

    def _imu_read_loop(self):
        while self.running:
            try:
                raw = self.bus.read_i2c_block_data(
                    MPU6886_ADDR, ACCEL_XOUT_H, 14
                )

                ax = self._int16(raw[0], raw[1]) / ACCEL_SCALE * G
                ay = self._int16(raw[2], raw[3]) / ACCEL_SCALE * G
                az = self._int16(raw[4], raw[5]) / ACCEL_SCALE * G

                gx = math.radians(
                    self._int16(raw[8], raw[9]) / GYRO_SCALE
                )
                gy = math.radians(
                    self._int16(raw[10], raw[11]) / GYRO_SCALE
                )
                gz = math.radians(
                    self._int16(raw[12], raw[13]) / GYRO_SCALE
                )

                with self.lock:
                    self.accel = [ax, ay, az]
                    self.gyro = [gx, gy, gz]

            except Exception as e:
                self.get_logger().warn(f"I2C read error: {e}")

            time.sleep(0.05)  # ~20 Hz sensor polling

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        with self.lock:
            msg.linear_acceleration.x = self.accel[0]
            msg.linear_acceleration.y = self.accel[1]
            msg.linear_acceleration.z = self.accel[2]

            msg.angular_velocity.x = self.gyro[0]
            msg.angular_velocity.y = self.gyro[1]
            msg.angular_velocity.z = self.gyro[2]

        msg.orientation_covariance[0] = -1.0 #orientation is unknown:

        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        self.publisher_.publish(msg)

    def _int16(self, high, low):
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

    def destroy_node(self):
        self.running = False
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MPU6886Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
