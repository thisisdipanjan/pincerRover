import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rpi5/pincer_ws/src/pincerRover/install/ackermann_drive_servo_arm'
