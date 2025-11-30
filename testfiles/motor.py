import smbus
import time

# I2C
I2C_ADDR = 0x34
bus = smbus.SMBus(1)   # Raspberry Pi I2C bus 1

# Register Addresses (same as your C code)
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_SPEED_ADDR = 51

# Motor types
MOTOR_TYPE_JGB37_520_12V_110RPM = 3

# Write array to I2C
def write_reg_array(reg, data_list):
    bus.write_i2c_block_data(I2C_ADDR, reg, data_list)

# ------------------------
# Initialization
# ------------------------
def init_motor_controller():
    print("Setting motor type...")
    write_reg_array(MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB37_520_12V_110RPM])
    time.sleep(0.01)

    print("Setting encoder polarity = 0...")
    write_reg_array(MOTOR_ENCODER_POLARITY_ADDR, [0])
    time.sleep(0.01)

    print("Motor controller ready.")

# ------------------------
# Drive Motors
# ------------------------
def drive_motors(speed_arr):
    """
    speed_arr = [m1, m2, m3, m4]
    Each speed is a signed value (similar to your C code)
    """
    write_reg_array(MOTOR_FIXED_SPEED_ADDR, speed_arr)

# ------------------------
# Example Run
# ------------------------
if __name__ == "__main__":
    init_motor_controller()

    print("Motors spinning forward...")

    # You must tune this value:
    # For HiWonder closed-loop motor control, ±50 is typical.
    motor_speed = 30  # adjust to reach ~60 RPM depending on your motor

    forward = [ motor_speed, motor_speed, motor_speed, motor_speed ]
    stop =    [ 0, 0, 0, 0 ]

    # Run forward 3 seconds
    drive_motors(forward)
    time.sleep(3)

    # Stop
    drive_motors(stop)
    time.sleep(2)

    print("Done.")
