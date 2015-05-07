import matplotlib.pyplot as plt
import serial
import time
import numpy as np 
import math
import threading
import sys

abort =False
ser = None
d_cm = 0
servo = 0

plt.ion()

def serial_reader():
    global d_cm,servo
    while True:
        if abort:
            break
        s = ser.readline()
        vals = s.split(" ")
        for v in vals:
            if v.find('d_cm')!=-1:
               data = float(v.split('=')[1])
               if data!=0:
                   d_cm = data
            if v.find('servo')!=-1:
               servo = float(v.split('=')[1])

servo_cmd = None
def cmd_reader():
    global servo_cmd
    while True:
        if abort:
            break
        cmd = raw_input('servo_cmd: ')
        ser.write(cmd)
        time.sleep(0.1)

try:
    ser = serial.Serial('/dev/cu.usbmodem1411',115200)
    ser.flushInput()
    t1 =  threading.Thread(target=serial_reader)
    t2 = threading.Thread(target=cmd_reader)
    t1.start()
    t2.start()

    plt.ion()
    plt.figure()
    l1, = plt.plot([],[],label='d_cm')
    l2, = plt.plot([],[],label='servo')

    plt.legend()
    plt.grid(True)
    plt.show()

    t = []
    d_cm_l = []
    servo_l = []

    prev_time = time.time()
    while True:
        if (time.time()-prev_time)>0.1:
            prev_time = time.time()
            print time.time(),d_cm,servo
            
            t.append(time.time())
            d_cm_l.append(d_cm)
            servo_l.append(servo)

            if len(t)>100:
                t.pop(0)
                d_cm_l.pop(0)
                servo_l.pop(0)
                
            l1.set_data(t,d_cm_l)
            l2.set_data(t,servo_l)

            plt.xlim(t[0],t[-1])
            plt.ylim(0,255)

            plt.draw()            
except Exception,e:
    print e
finally:
    abort = True
    ser.close()
    sys.exit()
