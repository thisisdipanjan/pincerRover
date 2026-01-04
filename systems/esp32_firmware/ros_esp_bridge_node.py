import serial, struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ESPBridge(Node):
    def __init__(self):
        super().__init__('esp_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

        self.imu_pub = self.create_publisher(Imu, 'imu/raw', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_cb, 10)

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        data = self.ser.read(64)
        if len(data) < 5 or data[0] != 0xAA:
            return

        msg_type = data[1]
        length = data[2]
        payload = data[3:3+length]

        if msg_type == 0x01:
            ax,ay,az,gx,gy,gz = struct.unpack('6h', payload)
            msg = Imu()
            msg.linear_acceleration.x = ax
            msg.angular_velocity.z = gz
            self.imu_pub.publish(msg)

        if msg_type == 0x02:
            x,y,th = struct.unpack('3f', payload)
            msg = Odometry()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            self.odom_pub.publish(msg)

    def cmd_cb(self, msg):
        steer = float(msg.angular.z)
        frame = struct.pack('<BBBfB', 0xAA, 0x10, 4, steer, 0)
        self.ser.write(frame)

def main():
    rclpy.init()
    rclpy.spin(ESPBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
