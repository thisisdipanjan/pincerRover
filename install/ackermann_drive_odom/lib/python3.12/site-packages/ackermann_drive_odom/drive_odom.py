import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos
import smbus
import struct
import time


class AckermannNode(Node):

    MOTOR_ADDR = 0x34
    SERVO_ADDR = 0x40

    ENC_BASE = 0x3C
    MOTOR_TYPE_ADDR = 0x14
    MOTOR_POLARITY_ADDR = 0x15
    MOTOR_SPEED_ADDR = 0x33

    MOTOR_TYPE_JGB37 = 3

    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    WHEEL_RADIUS = 0.034
    TICKS_PER_REV = 1316
    WHEEL_BASE = 0.175

    def __init__(self):
        super().__init__("ackermann_drive_with_odom")

        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = TransformBroadcaster(self)

        self.bus = smbus.SMBus(1)

        self.init_motors()

        self.init_servo()

        self.prev_left = 0
        self.prev_right = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.steering_angle = 95
        self.servo_ch = 0

        self.last_time = time.time()
        self.create_timer(0.02, self.update) 

        self.get_logger().info("Drive node running.")


    def write_motor(self, reg, data):
        self.bus.write_i2c_block_data(self.MOTOR_ADDR, reg, data)

    def read_encoder_raw(self, motor_id):
        addr = self.ENC_BASE + (motor_id * 4)
        raw = self.bus.read_i2c_block_data(self.MOTOR_ADDR, addr, 4)
        return struct.unpack('<i', bytes(raw))[0]


    def init_motors(self):
        self.write_motor(self.MOTOR_TYPE_ADDR, [self.MOTOR_TYPE_JGB37])
        time.sleep(0.01)

        self.write_motor(self.MOTOR_POLARITY_ADDR, [0])
        time.sleep(0.01)

        self.write_motor(self.ENC_BASE, [0] * 16)
        time.sleep(0.01)

    def init_servo(self, freq=50):
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x00)
        time.sleep(0.005)

        prescale_val = int(25000000.0 / (4096 * freq) - 1)
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x10)
        self.bus.write_byte_data(self.SERVO_ADDR, self.PRESCALE, prescale_val)
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x80)
        time.sleep(0.005)


    def set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.SERVO_ADDR, reg, on & 0xFF)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 1, on >> 8)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 3, off >> 8)

    def set_angle(self, channel, angle):
        pulse_min = 150
        pulse_max = 600
        pulse = int(pulse_min + (angle / 180.0) * (pulse_max - pulse_min))
        self.set_pwm(channel, 0, pulse)


    def drive_motors(self, speed):
        val = int(max(-100, min(100, speed)))
        self.write_motor(self.MOTOR_SPEED_ADDR, [val, val, val, val])


    def cmd_vel_callback(self, msg):
        linear = -msg.linear.x
        angular = -msg.angular.z

        motor_speed = int(linear * 50)
        self.drive_motors(motor_speed)

        if angular != 0:
            self.steering_angle += angular * 10
            self.steering_angle = max(40, min(150, self.steering_angle))
        else:
            self.steering_angle = 95

        self.set_angle(self.servo_ch, self.steering_angle)


    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        left_now = self.read_encoder_raw(2)
        right_now = self.read_encoder_raw(3)

        dl = left_now - self.prev_left
        dr = right_now - self.prev_right

        self.prev_left = left_now
        self.prev_right = right_now

        dist_l = (dl / self.TICKS_PER_REV) * (2 * 3.14159 * self.WHEEL_RADIUS)
        dist_r = (dr / self.TICKS_PER_REV) * (2 * 3.14159 * self.WHEEL_RADIUS)

        dist = (dist_l + dist_r) / 2
        dtheta = (dist_r - dist_l) / self.WHEEL_BASE

        self.theta += dtheta
        self.x += dist * cos(self.theta)
        self.y += dist * sin(self.theta)

        self.publish_odom()



    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta/2)
        odom.pose.pose.orientation.w = cos(self.theta/2)

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()