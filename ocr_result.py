#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageMsg
#from sensor_msgs.msg import CompressedImage

from std_msgs.msg import Int32

from PIL import Image
import pytesseract
from pytz import timezone
from datetime import datetime

from math import *


address_information = {"801":[24.0, -0.3], "802":[17.0, -0.3], "803":[8.0, 0.5], 
                        "804":[1.0, 0.0], "805":[-5.0, -0.5], "806":[2.4, -6.0], 
                        "807":[2.3, -23.0], "808":[3.0, -31.5], "809":[3.0, -40.0]}

max_ocr = 2

ocr_address = set()
address = []

class Gray():
    def __init__(self):
        ############################## USB_CAMERA & Recognize Address #################################
        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
        self.pub = rospy.Publisher('/ocr_result', Int32, latch=True, queue_size=10)
        self._sub = rospy.Subscriber('/usb_cam/image_raw', ImageMsg, self.callback, queue_size=10000)
              

    def callback(self, image_msg):
        if len(ocr_address) == max_ocr:
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Stop OCR Process")
            #raise rospy.ROSInterruptException
            #rospy.sleep(1)
        else:
            ########################## Recognize the address(room) using OCR ######################################      
            #np_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
            
            bridge = CvBridge()
            np_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_gray = cv2.cvtColor(np_image, cv2.COLOR_RGB2GRAY)
            imgThold = cv2.adaptiveThreshold(cv_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 1235, 3)
            resized_img_2 = cv2.resize(np_image, dsize=(800,500), interpolation=cv2.INTER_LINEAR) #full 1800, 1000
            
            

            try:
                text = pytesseract.image_to_string(imgThold, lang="kor")
            except ValueError:
                rospy.loginfo("Wrong")
            
            room = "호"
            room.decode('utf-8').encode("utf-8")
            if room in text:
                address_result = text.find(room)
                #rospy.loginfo(text[address_result-3:address_result])
                if (text[address_result-3]=="8") and  (1<int(text[address_result-1])<9): # and (int(text[address_result-2])==0)
                    try:
                        address_num = text[address_result-3:address_result]
                        #ocr_address.add(str(address_num))
                        if str(address_num) not in ocr_address:
                            rospy.loginfo(address_num)
                            ocr_address.add(str(address_num))
                            self.pub.publish(int(address_num))
                            #print(address_num, " is recognized.")
                        #else:
                            #print("Already recognized")
                            #rospy.sleep(2)
                    except ValueError:
                        rospy.loginfo("Wrong Address")
                        
            cv2.imshow('cv_gray', resized_img_2), cv2.waitKey(1)



    def main(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('gray', anonymous=True)
    
    try:
        node = Gray()
        node.main()
        rospy.sleep(2)
    except rospy.ROSInterruptException:
        pass
                                                                       
    
