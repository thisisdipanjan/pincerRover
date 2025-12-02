import time
import smbus

# I2C Setup
I2C_ADDR = 0x40       # Default PCA9685 address
bus = smbus.SMBus(1)

# PCA9685 Registers
MODE1          = 0x00
PRESCALE       = 0xFE
LED0_ON_L      = 0x06

# Initialize PCA9685
def init_pca9685(freq=50):
    bus.write_byte_data(I2C_ADDR, MODE1, 0x00)  # Normal mode
    time.sleep(0.005)

    prescale_val = int(25000000.0 / (4096 * freq) - 1)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x10)  # Sleep
    bus.write_byte_data(I2C_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x80)  # Restart
    time.sleep(0.005)

# Set PWM pulse
def set_pwm(channel, on, off):
    reg = LED0_ON_L + 4 * channel
    bus.write_byte_data(I2C_ADDR, reg, on & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg+1, on >> 8)
    bus.write_byte_data(I2C_ADDR, reg+2, off & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg+3, off >> 8)

# Set servo angle
def set_servo_angle(channel, angle):
    # 0° → 500µs pulse, 180° → 2500µs pulse
    pulse_min = 150    # 500µs
    pulse_max = 600    # 2500µs

    pulse = int(pulse_min + (angle / 180.0) * (pulse_max - pulse_min))
    set_pwm(channel, 0, pulse)

# Main demo
if __name__ == "__main__":
    print("Initializing PCA9685...")
    init_pca9685()

    CH = 0  # Channel 1

    print("Sweeping servo on channel 1...")

    while True:
        set_servo_angle(CH, 90)   # center
        time.sleep(1)

        set_servo_angle(CH, 120)  # right
        time.sleep(1)

        set_servo_angle(CH, 60)   # left
        time.sleep(1)
