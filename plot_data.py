import matplotlib.pyplot as plt
import serial
import time
import numpy as np 
import math

ser = None
try:
    ser = serial.Serial('/dev/cu.usbmodem1421',115200)
    #ser.close()
    #exit(0)

    plt.ion()
    plt.figure()

    l1, = plt.plot([],[],label='omega')
    l2, = plt.plot([],[],label='r_angle')
    l3, = plt.plot([],[],label='f_angle')

    plt.legend()
    plt.grid(True)
    plt.show()
    plt.draw()

    #ignores
    ser.flushInput()

    t = []
    omega_l = []
    r_angle_l = []
    f_angle_l = []
    ROutput_l = []
    LOutput_l = []

    prev_time = time.time()
    while True:
        s = ser.readline()

        if (time.time()-prev_time)>0.1:
            prev_time = time.time()
            vals = s.split()

            print vals,np.median(omega_l),np.median(r_angle_l),np.median(f_angle_l)

            omega = 0
            r_angle = 0
            f_angle = 0
            LOutput = 0
            ROutput = 0
            kp = 0
            ki = 0

            for v in vals:
                if v.find('omega')!=-1:
                   omega = float(v.split('=')[1])
                if v.find('r_angle')!=-1:
                   r_angle = float(v.split('=')[1])
                if v.find('f_angle')!=-1:
                   f_angle = float(v.split('=')[1])
                if v.find('LOutput')!=-1:
                   LOutput = float(v.split('=')[1])
                if v.find('ROutput')!=-1:
                   ROutput = float(v.split('=')[1])
                if v.find('kp')!=-1:
                   kp = float(v.split('=')[1])
                if v.find('ki')!=-1:
                   ki = float(v.split('=')[1])

            # print omega,r_angle,f_angle,kp,ki,LOutput,ROutput 
            
            t.append(time.time())
            omega_l.append(omega)
            r_angle_l.append(r_angle)
            f_angle_l.append(f_angle)
            
            if len(omega_l)>100:
                t.pop(0)
                omega_l.pop(0)
                r_angle_l.pop(0)
                f_angle_l.pop(0)

            #l1.set_data(t,omega_l)
            l2.set_data(t,r_angle_l)
            l3.set_data(t,f_angle_l)
            plt.xlim(t[0],t[-1])
            plt.ylim(-20,20)

            plt.draw()

except Exception,e:
    print e
finally:
    ser.close()
