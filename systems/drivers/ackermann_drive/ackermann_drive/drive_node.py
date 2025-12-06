import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus
import time

bus = smbus.SMBus(1)
MOTOR_I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_TYPE_JGB37_520_12V_110RPM = 3
SERVO_I2C_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

def motor_write(reg, data_list):
    bus.write_i2c_block_data(MOTOR_I2C_ADDR, reg, data_list)


def init_motor():
    motor_write(MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB37_520_12V_110RPM])
    time.sleep(0.01)
    motor_write(MOTOR_ENCODER_POLARITY_ADDR, [0])
    time.sleep(0.01)


def drive_motors(m1, m2, m3, m4):
    motor_write(MOTOR_FIXED_SPEED_ADDR, [m1, m2, m3, m4])


def init_servo(freq=50):
    bus.write_byte_data(SERVO_I2C_ADDR, MODE1, 0x00)
    time.sleep(0.005)

    prescale_val = int(25000000.0 / (4096 * freq) - 1)
    bus.write_byte_data(SERVO_I2C_ADDR, MODE1, 0x10)
    bus.write_byte_data(SERVO_I2C_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(SERVO_I2C_ADDR, MODE1, 0x80)
    time.sleep(0.005)


def set_pwm(channel, on, off):
    reg = LED0_ON_L + 4 * channel
    bus.write_byte_data(SERVO_I2C_ADDR, reg, on & 0xFF)
    bus.write_byte_data(SERVO_I2C_ADDR, reg + 1, on >> 8)
    bus.write_byte_data(SERVO_I2C_ADDR, reg + 2, off & 0xFF)
    bus.write_byte_data(SERVO_I2C_ADDR, reg + 3, off >> 8)


def set_angle(channel, angle):
    pulse_min = 150
    pulse_max = 600
    pulse = int(pulse_min + (angle / 180.0) * (pulse_max - pulse_min))
    set_pwm(channel, 0, pulse)


class AckermannDriveNode(Node):

    def __init__(self):
        super().__init__('ackermann_drive')

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        init_motor()
        init_servo()

        self.get_logger().info("PincerRover node executing.")
        self.servo_ch = 0
        self.steering = 90

    
    def cmd_vel_callback(self, msg):
        linear = -msg.linear.x       
        angular = -msg.angular.z  
        speed = int(linear * 50)
        drive_motors(speed, speed, speed, speed)

        self.steering += (angular*10)
        if angular!=0:
            self.steering = max(40, min(150, self.steering))
        else:
            self.steering = 95
        set_angle(self.servo_ch, self.steering)

        self.get_logger().info(
            f"speed={speed}, steering_angle={self.steering}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()