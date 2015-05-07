import serial
import time
import numpy as np 
import math
import threading
import sys
import tf

abort =False
ser = None
d_cm = 0
servo = 90
servo_cmd = 90

def serial_reader():
    global d_cm,servo
    while not rospy.is_shutdown():
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

def cmd_reader():
    global servo_cmd
    while not rospy.is_shutdown():
        if abort:
            break
        servo_cmd = int(raw_input('servo_cmd: '))
        if servo_cmd!=-1:
            ser.write(str(servo_cmd))
            time.sleep(0.3)
        else:
            iteration = 2
            for i in range(iteration):
                for j in range(0,180,5)+range(180,0,-5):
                    servo_cmd = int(j)
                    ser.write(str(servo_cmd))
                    time.sleep(0.2)
import rospy
from sensor_msgs.msg import Range
try:
    rospy.init_node('sonar')
    pub = rospy.Publisher('sonar',Range,queue_size=10)
    
    ser = serial.Serial('/dev/cu.usbmodem1411',115200)
    ser.flushInput()
    t1 =  threading.Thread(target=serial_reader)
    t2 = threading.Thread(target=cmd_reader)
    t1.start()
    t2.start()
    
    r_msg = Range()
    r_msg.radiation_type = r_msg.ULTRASOUND
    r_msg.field_of_view = 0.1
    r_msg.min_range = 0.1
    r_msg.max_range = 2.0
    r_msg.header.frame_id = 'sonar_link'

    r= rospy.Rate(20)
    while not rospy.is_shutdown():
        yaw = (90-servo_cmd)/180.0*3.14
        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,0),
                tf.transformations.quaternion_from_euler(0,0,yaw),
                rospy.Time.now(),"world","sonar_link")
        
        r_msg.range = d_cm/100.0
        r_msg.header.stamp = rospy.Time.now()
        pub.publish(r_msg)
        
        r.sleep()

except Exception,e:
    print e
finally:
    abort = True
    ser.close()
    sys.exit()
