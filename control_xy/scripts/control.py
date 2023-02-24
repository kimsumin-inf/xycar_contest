#!/usr/bin/env python3
import sys
import rospy
import roslib 
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from xycar_msgs.msg import xycar_motor

class Frame_CB:
    def __init__(self):
        self.sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        self.angle = 0
        self.velocity = 0

    def callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.resize(image_np, (360,360))
        cv2.imshow("frame" , image_np)
        ch = cv2.waitKey(1)
        if ch ==ord("a"):
            self.angle -=10
            if self.angle <-80:
                self.angle =-80
        if ch == ord("d"):
            self.angle +=10
            if self.angle > 80:
                self.angle =80
        if ch == ord("w"):
            self.velocity +=5
            if self.velocity >25:
                self.velocity =25

        if ch == ord("x"):
            self.velocity -=5
            if self.velocity < -25:
                self.velocity =-25
        if ch == ord("s"):
            self.angle = 0
            self.velocity =0
        print(f"angle : {self.angle}, velocity : {self.velocity}")
        motor = xycar_motor()
        motor.angle = self.angle
        motor.speed = self.velocity
        self.pub.publish(motor)
def main(argv):
    Frame = Frame_CB()
    rospy.init_node("control", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutdown")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
