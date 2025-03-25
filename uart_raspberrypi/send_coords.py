#!/usr/bin/env python
import time
import serial
import random


ser = serial.Serial(
        port='/dev/ttyAMA0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None
)

while 1:
        x=ser.readline()
        print(x)
        time.sleep(0.01)
        # Generate two random integer coordinates
        size = random.randint(1, 5)
        x, y = random.randint(0, 255), random.randint(0, 255)
        data = bytearray([x, y])
        for i in range(size-1):
                x, y = random.randint(0, 255), random.randint(0, 255)
                data += bytearray([x, y])
        ser.write(bytes([size]))
        time.sleep(0.10)
        ser.write(data)
