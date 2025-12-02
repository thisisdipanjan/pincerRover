import time
import smbus

I2C_ADDR = 0x48            # ADS1115 default address
bus = smbus.SMBus(1)

# ADS1115 Registers
REG_CONVERT = 0x00
REG_CONFIG  = 0x01

# PGA gain values
ADS1115_PGA_4_096V = 0x0200   # ±4.096V range

# OS Bit (start single conversion)
OS_SINGLE = 0x8000

# MUX Channels
MUX_A0 = 0x4000   # AIN0
MUX_A1 = 0x5000   # AIN1
MUX_A2 = 0x6000   # AIN2

# Mode
MODE_SINGLE = 0x0100

# Sampling rate
DR_128SPS = 0x0080

# Comparator off
COMP_DISABLE = 0x0003

def read_channel(mux):
    # Build config register value
    config = (
        OS_SINGLE     |   # Start conversion
        mux           |   # Channel selection
        ADS1115_PGA_4_096V |
        MODE_SINGLE   |
        DR_128SPS     |
        COMP_DISABLE
    )

    # Write config to start conversion
    bus.write_i2c_block_data(I2C_ADDR, REG_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])

    # Wait for conversion (8ms for 128SPS)
    time.sleep(0.009)

    # Read conversion result
    data = bus.read_i2c_block_data(I2C_ADDR, REG_CONVERT, 2)
    raw = (data[0] << 8) | data[1]

    # Convert from signed 16-bit
    if raw > 32767:
        raw -= 65536

    # Convert raw ADC to voltage (depending on PGA)
    voltage = raw * (4.096 / 32768.0)

    return raw, voltage


if __name__ == "__main__":
    print("Reading ADS1115 A0, A1, A2...")

    while True:
        raw0, v0 = read_channel(MUX_A0)
        raw1, v1 = read_channel(MUX_A1)
        raw2, v2 = read_channel(MUX_A2)

        print(f"A0: {v0:.4f} V   (raw={raw0})")
        print(f"A1: {v1:.4f} V   (raw={raw1})")
        print(f"A2: {v2:.4f} V   (raw={raw2})")
        print("-" * 40)

        time.sleep(0.5)
