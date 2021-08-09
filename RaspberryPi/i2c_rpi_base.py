from smbus import SMBus
from MovementModule import movement

# Address of slave device (Arduino)
addr = 0x8
# Reference to I2C port 1
bus = SMBus(1)

print("Enter 1eft speed and right speed")

while True:
    # Get left and right wheel speeds as input
    lSpeed, rSpeed = map(int, input(">>>>  ").split())
    # Write the speed data to the bus
    bus.write_i2c_block_data(addr, lSpeed, [rSpeed])
