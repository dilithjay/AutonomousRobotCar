from smbus import SMBus
from threading import Timer


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
        print(turn_amount)

    def change_turn_amount(self, amount):
        self.turn_amount += amount
        return self.turn_amount

    def set_delayed_turn_amount(self, delay, turn_amount):
        """
        Apply the turn amount to the wheels after a specified delay.

        :param delay: Amount of time by which the applying of the turn amount should be delayed.
        :param turn_amount: Turn amount to be applied.
        """
        timer = Timer(delay, self.set_turn_amount, [turn_amount])
        timer.start()

    def set_speed(self, speed):
        """Set the overall speed of the robot."""
        self.speed = speed

    def apply_speeds(self):
        """Method to set the motor rotation speeds."""
        l_speed, r_speed = self.get_speeds()

        # Write the speed data to the I2C bus
        try:
            self.bus.write_i2c_block_data(self.address, r_speed, [l_speed])
        except OSError as e:
            print("Connection to Arduino lost:", e)

    def reset_speeds(self):
        """Reset the wheel speeds to 0."""
        self.bus.write_i2c_block_data(self.address, 0, [0])
