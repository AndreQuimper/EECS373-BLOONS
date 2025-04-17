#!/usr/bin/env python
import time
import serial
import random
import threading

mutex = threading.Lock()
cnt = 0
coords = []

ser = serial.Serial(
        port='/dev/ttyAMA0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None
)

def display():
    global cnt 
    global coords
    while 1:
        mutex.acquire()
        size = cnt
        my_coords = coords 
        mutex.release()

        x=ser.readline()
        print(x)
        data = bytearray(my_coords)
        ser.write(bytes([size]))
        ser.write(data)

def gen():
    global cnt 
    global coords
    while 1:
        my_cnt = random.randint(1, 10)
        my_coords = []
        for i in range(my_cnt):
            my_coords.append(random.randint(0, 255))
        mutex.acquire()
        cnt = my_cnt
        coords = my_coords
        mutex.release()
        time.sleep(random.randint(1, 8))

t1 = threading.Thread(target=display, args=())
t2 = threading.Thread(target=gen, args=())
t2.start()
t1.start()
t2.join()
