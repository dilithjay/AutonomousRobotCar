from smbus import SMBus


class Movement:
    def __init__(self, start_speed=200, turn_amount=0, arduino_address=0x8):
        """
        Initialize the movement.

        :param start_speed: Initial average speed of wheels
        :param turn_amount: A measure of how much the car should turn to the right (Range -128 to 128)
        :param arduino_address: I2C address of the slave device (Arduino)
        """
        self.address = 0x8
        self.bus = SMBus(1)
        self.speed = start_speed
        self.turn_amount = turn_amount

    def get_speeds(self):
        """Calculate, clamp and return the left and right wheel speeds."""
        l_speed = max(min(self.speed + self.turn_amount, 255), 0)
        r_speed = max(min(self.speed - self.turn_amount, 255), 0)
        return l_speed, r_speed

    def set_turn_amount(self, turn_amount):
        """Set the turn amount"""
        self.turn_amount = turn_amount

    def set_speed(self, speed):
        """Set the overall speed of the robot."""
        self.speed = speed

    def apply_speeds(self):
        """Method to set the motor rotation speeds."""
        l_speed, r_speed = self.get_speeds()

        # Write the speed data to the I2C bus
        self.bus.write_i2c_block_data(self.address, l_speed, [r_speed])

    def reset_speeds(self):
        """Reset the wheel speeds to 0."""
        self.bus.write_i2c_block_data(self.address, 0, [0])

