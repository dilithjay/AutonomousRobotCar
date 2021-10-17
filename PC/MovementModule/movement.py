import serial
from threading import Timer
from time import time


class Movement:
    def __init__(self, start_speed=200, turn_amount=0, interval=0.5, calibration=0):
        self.ser = serial.Serial(port='COM4', baudrate=9600, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
        self.speed = start_speed
        self.turn_amount = turn_amount
        self.interval = interval
        self.next_check_time = time()
        self.calibration = calibration

    def get_speeds(self):
        """Calculate, clamp and return the left and right wheel speeds."""
        l_speed = max(min(self.speed + self.turn_amount, 255), 0) - self.calibration
        r_speed = max(min(self.speed - self.turn_amount, 255), 0) + self.calibration
        # print("l r =", l_speed, r_speed)
        return l_speed, r_speed

    def set_turn_amount(self, turn_amount):
        """Set the turn amount"""
        self.turn_amount = turn_amount

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
        if time() < self.next_check_time:
            return
        self.next_check_time = time() + self.interval
        l_speed, r_speed = self.get_speeds()
        self.ser.write(bytes([l_speed, r_speed]))
        print("==============================")
        print(self.ser.read(1024).decode())

    def reset_speeds(self):
        """Reset the wheel speeds to 0."""
        self.ser.write(bytes([0, 0]))
