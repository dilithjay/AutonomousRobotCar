import serial
from time import sleep

serial_port = serial.Serial(port='COM4', baudrate=9600, timeout=0, stopbits=1)
serial_port.flushInput()
serial_port.flushOutput()
serial_port.flush()
print("Connected")
size = 1024

while True:
    inp = input(": ")
    if inp.lower() == 'q':
        break
    serial_port.write(bytes(list(map(int, inp.split()))))
    sleep(.1)
    data = serial_port.read(size).decode('ascii')
    while not data:
        data = serial_port.read(size).decode('ascii')
    sleep(.1)
    if data:
        print(data)
serial_port.write(bytes([0, 0]))

# 0 0
