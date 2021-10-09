import bluetooth
from threading import Timer
from time import time


def get_bd_address(bd_name):
    print("Searching for devices...")
    nearby_devices = bluetooth.discover_devices()
    print("Devices found.")

    for device in nearby_devices:
        if bluetooth.lookup_name(device) == bd_name:
            print("Bluetooth Device found!")
            return device
    return None


class Movement:
    def __init__(self, start_speed=200, turn_amount=0, bd_name='HC-05', port=1, interval=0.5):
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        bd_addr = get_bd_address(bd_name)
        self.sock.connect((bd_addr, port))
        self.speed = start_speed
        self.turn_amount = turn_amount
        self.interval = interval
        self.next_check_time = time()

    def get_speeds(self):
        """Calculate, clamp and return the left and right wheel speeds."""
        l_speed = max(min(self.speed + self.turn_amount, 255), 0)
        r_speed = max(min(self.speed - self.turn_amount, 255), 0)
        # print("l r =", l_speed, r_speed)
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
        if time() < self.next_check_time:
            return
        self.next_check_time = time() + self.interval
        l_speed, r_speed = self.get_speeds()
        self.sock.send(bytes([l_speed, r_speed]))
        self.sock.rfcomm_read_msg()

    def reset_speeds(self):
        """Reset the wheel speeds to 0."""
        self.sock.send(bytes([0, 0]))
        self.sock.close()
