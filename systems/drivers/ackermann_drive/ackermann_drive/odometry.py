import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf_transformations
from geometry_msgs.msg import TransformStamped
import smbus
import time
import math

bus = smbus.SMBus(1)

MOTOR_I2C_ADDR = 0x34
ENC1_ADDR = 0x18
ENC2_ADDR = 0x1C
ENC3_ADDR = 0x20
ENC4_ADDR = 0x24

WHEEL_RADIUS = 0.033 
WHEEL_BASE = 0.25      
COUNTS_PER_REV = 215

class OdometryNode(Node):

    def __init__(self):
        super().__init__("odom_publisher")

        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TransformStamped, "/tf", 10)
        self.subscription = self.create_subscription(
            Float32, "/steering_angle", self.steering_callback, 10
        )

        self.steering_angle = 90  # default center
        self.prev_enc = self.read_all_enc()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = time.time()
        self.create_timer(0.02, self.update_odometry)  # 50Hz update


    def steering_callback(self, msg):
        self.steering_angle = msg.data


    def read_encoder_32(self, addr):
        data = bus.read_i2c_block_data(MOTOR_I2C_ADDR, addr, 4)
        raw = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
        if raw & 0x80000000:  
            raw -= 0x100000000
        return raw


    def read_all_enc(self):
        return [
            self.read_encoder_32(ENC1_ADDR),
            self.read_encoder_32(ENC2_ADDR),
            self.read_encoder_32(ENC3_ADDR),
            self.read_encoder_32(ENC4_ADDR),
        ]


    def update_odometry(self):

        current_enc = self.read_all_enc()
        dt = time.time() - self.last_time
        self.last_time = time.time()

        # delta encoder
        dE = [current_enc[i] - self.prev_enc[i] for i in range(4)]
        self.prev_enc = current_enc

        # distance per wheel
        dist_per_count = (2 * math.pi * WHEEL_RADIUS) / COUNTS_PER_REV
        wheel_dist = [d * dist_per_count for d in dE]

        # average distance of all wheels
        d = sum(wheel_dist) / 4.0

        # Convert steering angle to radians
        steering_rad = math.radians(self.steering_angle - 90)

        # Ackermann turning
        if abs(steering_rad) < 0.01:  # nearly straight
            dx = d * math.cos(self.yaw)
            dy = d * math.sin(self.yaw)
            dyaw = 0.0
        else:
            turning_radius = WHEEL_BASE / math.tan(steering_rad)
            dyaw = d / turning_radius
            dx = turning_radius * (math.sin(self.yaw + dyaw) - math.sin(self.yaw))
            dy = -turning_radius * (math.cos(self.yaw + dyaw) - math.cos(self.yaw))

        # update global pose
        self.x += dx
        self.y += dy
        self.yaw += dyaw

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.publisher.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
