import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math

I2C_ADDR = 0x68
bus = smbus.SMBus(1)

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu")

        self.pub = self.create_publisher(Imu, "/imu", 10)
        self.timer = self.create_timer(0.02, self.read_imu)

        # Wake IMU
        bus.write_byte_data(I2C_ADDR, PWR_MGMT_1, 0)

        self.get_logger().info("IMU Node started")

    def read_word(self, reg):
        high = bus.read_byte_data(I2C_ADDR, reg)
        low  = bus.read_byte_data(I2C_ADDR, reg + 1)
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    def read_imu(self):
        msg = Imu()

        ax = self.read_word(ACCEL_XOUT_H) / ACCEL_SCALE
        ay = self.read_word(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self.read_word(ACCEL_XOUT_H + 4) / ACCEL_SCALE

        gx = self.read_word(GYRO_XOUT_H) / GYRO_SCALE
        gy = self.read_word(GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = self.read_word(GYRO_XOUT_H + 4) / GYRO_SCALE

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
