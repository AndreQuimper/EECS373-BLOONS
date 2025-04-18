# Required Peripheral Libraries
import cv2
import imutils
import time
import serial
import random
import threading
import copy

# Initialize the camera object
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.start()

#setting color bounds - set for green right now. Use HSV values
#Need to set up for multiple colors and try to narrow down the color ranges

dataLock = threading.Lock()
threadCv = threading.Condition()
listUpdated = False
cnt = 0
coords = []

ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None
)

def display():
    global cnt 
    global coords
    global listUpdated
    global threadCv
    while 1:
        x=ser.readline()
        print(x)
        threadCv.acquire()
        while not listUpdated:
            threadCv.wait()
        listUpdated = False
        threadCv.release()
        dataLock.acquire()
        size = cnt
        my_coords = copy.deepcopy(coords)
        dataLock.release()
        data = b""
        print(len(my_coords))
        for bloon in my_coords:
            bytedata = bytes(bloon["color"],'ascii')[:1] + bloon["x_axis"].to_bytes(1,'little') + bloon["y_axis"].to_bytes(1,'little')
            data += bytedata
            print(bloon["color"], bloon["x_axis"], bloon["y_axis"])
            
        ser.write(bytes([size]))
        ser.write(data)
        

colRanges = {
    'G': ((35, 86, 96), (50, 255, 255)),
    'B': ((95, 196, 196), (120, 255, 255)),
    'R': ((130, 128, 128), (176, 255, 255))
    #'O': ((5, 100, 100), (25, 255, 255))
}

#picam2.set_controls({"AwbMode": "off"})
# picam2.set_controls({"AwbGainRed": 1.5, "AwbGainBlue": 1.5})

def gen():
    global cnt
    global coords
    global listUpdated
    global threadCv
    while True:
        #CV type splash
        frame = picam2.capture_array()
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = imutils.resize(frame, width=600)
        blur = cv2.GaussianBlur(frame, (11,11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        count = 0
        detectedCircs = []
        for color, (lower, upper) in colRanges.items():
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow("mask", mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
    
            if len(cnts) > 0:
                
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                x_coord = int((int(M["m10"] / M["m00"]) / frame.shape[1]) * 255)
                y_coord = int((int(M["m01"] / M["m00"]) / frame.shape[0]) * 255)
                #print(radius)
                if radius > 25:
                    count = count + 1
                    circData = {
                        "color": color,
                        "x_axis": x_coord,
                        "y_axis": y_coord
                    }
                    detectedCircs.append(circData)
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    cv2.putText(frame, color, (int(x)-10,int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    #print(hsv[int(frame.shape[1]/2), int(frame.shape[0]/2)])
            dataLock.acquire()
            cnt = count
            coords = copy.deepcopy(detectedCircs)
            dataLock.release()
            threadCv.acquire()
            listUpdated = True
            threadCv.notify()
            threadCv.release()
        #Yay circles
        cv2.putText(frame, str(detectedCircs), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 5, (0, 0, 255), -1)
        cv2.circle(hsv, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 5, (0, 0, 255), -1)
        cv2.imshow("circles", frame)
        #cv2.imshow("HSV", hsv)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cv2.destroyAllWindows()
    picam2.stop()



t1 = threading.Thread(target=display, args=())
t2 = threading.Thread(target=gen, args=())
t2.start()
t1.start()
t2.join()