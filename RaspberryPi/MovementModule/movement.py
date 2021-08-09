from smbus import SMBus


class Movement:
    def __init__(self, arduino_address=0x8, start_lspeed=200, start_rspeed=200):
        """
        Initialize the movement.

        :param arduino_address: I2C address of the slave device (Arduino)
        :param start_lspeed: Initial speed of left wheel
        :param start_rspeed: Initial speed of right wheel
        """
        self.addr = 0x8
        self.bus = SMBus(1)
        self.l_speed = start_lspeed
        self.r_speed = start_rspeed

    def get_speed(self):
        """
        Method to get the left and right wheel speeds.

        :return: left speed, right speed
        """
        return self.l_speed, self.r_speed

    def set_speed(self, l_speed, r_speed):
        """
        Method to set the left and right wheel speeds.

        :param l_speed: Left Speed
        :param r_speed: Right Speed
        """

        # Temporary. Apply PID control if necessary
        self.l_speed = l_speed
        self.r_speed = r_speed
        # Write the speed data to the I2C bus
        self.bus.write_i2c_block_data(self.addr, l_speed, [r_speed])

