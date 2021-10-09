import serial
from time import sleep

serial_port = serial.Serial(port='COM4', baudrate=9600, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024

while True:
    serial_port.write(bytes(list(map(int, input().split()))))
    sleep(.05)
    data = serial_port.read(size).decode('ascii')
    while not data:
        data = serial_port.read(size).decode('ascii')
    sleep(.01)
    if data:
        print(data)
