import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import smbus2 as smbus
import threading
import queue
import time
import errno


class AckermannNode(Node):

    MOTOR_ADDR = 0x34
    SERVO_ADDR = 0x40
    MOTOR_TYPE_ADDR = 0x14
    MOTOR_POLARITY_ADDR = 0x15
    MOTOR_SPEED_ADDR = 0x33
    MOTOR_TYPE_JGB37 = 3
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    STEERING_CH = 0
    WAIST_CH = 7
    SHOULDER_CH = 3
    WRIST_CH = 15
    GRIPPER_CH = 4

    SERVO_LIMITS = {
        "steering": (40.0, 150.0, 95.0),
        "waist":    (0.0, 160.0, 70.0),
        "shoulder": (10.0, 160.0, 110.0),
        "wrist":    (60.0, 165.0, 165.0),
        "gripper":  (60.0, 70.0, 65.0), 
    }

    def __init__(self):
        super().__init__("ackermann_drive_controller")

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(Float32, "/waist_servo",    self.waist_cb, 10)
        self.create_subscription(Float32, "/shoulder_servo", self.shoulder_cb, 10)
        self.create_subscription(Float32, "/wrist_servo",    self.wrist_cb, 10)
        self.create_subscription(Float32, "/gripper_servo",  self.gripper_cb, 10)

        self.bus = smbus.SMBus(1)
        self.i2c_lock = threading.Lock()
        self.i2c_queue = queue.Queue()
        self.i2c_ok = True

        self.i2c_thread = threading.Thread(
            target=self._i2c_worker, daemon=True
        )
        self.i2c_thread.start()

        self._enqueue(self._init_motors)
        self._enqueue(self._init_servo)
        self.steering_angle = 95
        for name, (_, _, center) in self.SERVO_LIMITS.items():
            ch = getattr(self, f"{name.upper()}_CH")
            self._enqueue(self._set_angle, ch, center)

        self.get_logger().info("Ackermann control node started")


    def _enqueue(self, fn, *args):
        self.i2c_queue.put((fn, args))

    def _i2c_worker(self):
        while rclpy.ok():
            try:
                fn, args = self.i2c_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                with self.i2c_lock:
                    fn(*args)
            except OSError as e:
                if e.errno in (errno.ETIMEDOUT, errno.EIO):
                    self.i2c_ok = False
                    self.get_logger().error(
                        f"I2C bus error ({e.errno}): {e}. Device not responding."
                    )
                else:
                    self.get_logger().error(f"I2C error: {e}")
            except Exception as e:
                self.get_logger().error(f"I2C unexpected error: {e}")


    def _init_motors(self):
        self.bus.write_i2c_block_data(
            self.MOTOR_ADDR,
            self.MOTOR_TYPE_ADDR,
            [self.MOTOR_TYPE_JGB37]
        )
        time.sleep(0.01)

        self.bus.write_i2c_block_data(
            self.MOTOR_ADDR,
            self.MOTOR_POLARITY_ADDR,
            [0]
        )
        time.sleep(0.01)

        self.get_logger().info("Motor driver initialized")

    def _init_servo(self, freq=50):
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x00)
        time.sleep(0.005)

        prescale = int(25_000_000 / (4096 * freq) - 1)
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x10)
        self.bus.write_byte_data(self.SERVO_ADDR, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.SERVO_ADDR, self.MODE1, 0x80)
        time.sleep(0.005)

        self.get_logger().info("Servo controller initialized")


    def _set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.SERVO_ADDR, reg, on & 0xFF)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 1, on >> 8)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.SERVO_ADDR, reg + 3, off >> 8)

    def _set_angle(self, channel, angle_deg):
        pulse_min = 150
        pulse_max = 600
        angle = max(0.0, min(180.0, angle_deg))
        pulse = int(pulse_min + (angle / 180.0) * (pulse_max - pulse_min))
        self._set_pwm(channel, 0, pulse)

    def _drive_motors(self, speed):
        val = int(max(-100, min(100, speed)))
        self.bus.write_i2c_block_data(
            self.MOTOR_ADDR,
            self.MOTOR_SPEED_ADDR,
            [val, val, val, val]
        )


    def cmd_vel_cb(self, msg):
        self._enqueue(self._drive_motors, int(-msg.linear.x * 50))
        mn, mx, center = self.SERVO_LIMITS["steering"]
        angular = -msg.angular.z

        if angular != 0:
            self.steering_angle += angular * 10
            self.steering_angle = max(mn, min(mx, self.steering_angle))
        else:
            self.steering_angle = center

        self._enqueue(self._set_angle, self.STEERING_CH, self.steering_angle)

    def waist_cb(self, msg):
        self._servo_cmd(self.WAIST_CH, msg.data, "waist")

    def shoulder_cb(self, msg):
        self._servo_cmd(self.SHOULDER_CH, msg.data, "shoulder")

    def wrist_cb(self, msg):
        self._servo_cmd(self.WRIST_CH, msg.data, "wrist")

    def gripper_cb(self, msg):
        self._servo_cmd(self.GRIPPER_CH, msg.data, "gripper")

    def _servo_cmd(self, channel, angle, name):
        mn, mx, _ = self.SERVO_LIMITS[name]
        angle = max(mn, min(mx, angle))
        # print(angle)
        self._enqueue(self._set_angle, channel, angle)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()