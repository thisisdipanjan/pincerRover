#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys, termios, tty

KEY_BINDINGS = {
    'g': ('gripper_servo', 1.0),
    'h': ('gripper_servo', -1.0),
    's': ('shoulder_servo', 1.0),
    'a': ('shoulder_servo', -1.0),
    'w': ('waist_servo', 1.0),
    'q': ('waist_servo', -1.0),
    'r': ('wrist_servo', 1.0),
    'e': ('wrist_servo', -1.0),
}

class ManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pubs = {
            'gripper_servo': self.create_publisher(Float32, '/gripper_servo', 10),
            'shoulder_servo': self.create_publisher(Float32, '/shoulder_servo', 10),
            'waist_servo': self.create_publisher(Float32, '/waist_servo', 10),
            'wrist_servo': self.create_publisher(Float32, '/wrist_servo', 10),
        }
        self.joint_values = {
            'gripper_servo': 0.0,
            'shoulder_servo': 0.0,
            'waist_servo': 0.0,
            'wrist_servo': 0.0,
        }
        self.increment = 1.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        print("Control the 4-DOF arm:")
        print("Gripper: g/h | Shoulder: s/a | Waist: w/q | Wrist: r/e | Ctrl-C to quit")
        while True:
            key = self.get_key()
            if key == '\x03':  # Ctrl-C
                print("\nExiting...")
                break

            if key in KEY_BINDINGS:
                joint_name, delta = KEY_BINDINGS[key]
                self.joint_values[joint_name] += delta * self.increment
                # Optional: clamp values if needed
                self.joint_values[joint_name] = max(min(self.joint_values[joint_name], 180.0), 0.0)
                msg = Float32()
                msg.data = self.joint_values[joint_name]
                self.pubs[joint_name].publish(msg)
                print(f"{joint_name}: {self.joint_values[joint_name]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
