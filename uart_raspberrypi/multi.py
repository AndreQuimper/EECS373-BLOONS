import time 
import random
import threading

mutex = threading.Lock()
cnt = 0
coords = []

def display():
    global cnt 
    global coords
    while 1:
        mutex.acquire()
        my_cnt = cnt
        my_coords = coords 
        mutex.release()
        print(my_cnt)
        print(my_coords)
        time.sleep(5)

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
