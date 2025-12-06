import smbus2
import struct
import time

ADDR = 0x34
bus = smbus2.SMBus(1)

ENC_BASE = 0x3C

def read_encoder(motor):
    addr = ENC_BASE + (motor * 4)
    raw = bus.read_i2c_block_data(ADDR, addr, 4)
    value = struct.unpack('<i', bytes(raw))[0]
    return value

while True:
    left  = read_encoder(2)   # Motor1
    right = read_encoder(3)   # Motor2
    print("Left:", left, "\tRight:", right)
    time.sleep(0.1)
