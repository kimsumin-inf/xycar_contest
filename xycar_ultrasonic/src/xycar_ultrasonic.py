#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : xycar_ultrasonic.py
# 버 전 : ultrasonic.8.v1.1
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import serial, rospy
from std_msgs.msg import Int32MultiArray

class ultrasonic_pub:
    ser = serial.Serial(port='/dev/ttySONIC', baudrate=38400,)
    US = [0,0,0,0,0,0,0,0,0]
 
    def __init__(self):
        self.DEBUG = False
        self.DEBUGERRMSG = [
            "Received data has been corrupted. (-3)",
            "Received data has been corrupted. (-2)",
            "No data received. (-1)"
        ]
        self.pub = rospy.Publisher('xycar_ultrasonic', Int32MultiArray, queue_size=1)
        self.ultra_sonic = Int32MultiArray()
 
    def send(self):
        for i in range(0,8):
            idx, val, err, self.US[0] = 0, 0, 0, 0
            idx, val, err = self.read_value(self.ser.readline())

            self.US[idx] = val

            if self.DEBUG:
                if self.US[0] < 0:
                    print("ULTRASONIC : ", self.DEBUGERRMSG[self.US[0]], " location : ", i, " value : ", errF)

        self.ser.flushInput()
 
        self.ultra_sonic.data = [self.US[1], self.US[2], self.US[3], self.US[4], self.US[5], self.US[6], self.US[7], self.US[8]]

        if self.DEBUG:
            print(self.ultra_sonic.data)

        self.pub.publish(self.ultra_sonic)
 
    def read_value(self, serial_value):
        stri = serial_value[:-2].decode('ascii')
        if len(stri) == 0:
            return 0, -1, True

        number = ""
        for word in stri:
            if word.isdigit():
                number += word
                continue

        if len(stri) != len(number):
            return 0, -2, len(stri) - len(number)
 
        int_serial_value = int(number)
 
        value_number = int_serial_value % 10
        value = int_serial_value // 10
 
        if (value_number > 8) or (value_number < 1):
            return 0, -3, value_number
 
        return value_number, value, False

    def close(self):
        self.ser.close()

rospy.init_node('xycar_ultrasonic', anonymous=False)
ultrasonic = ultrasonic_pub()  
 
while not rospy.is_shutdown():
   ultrasonic.send()

ultrasonic.close()
        
