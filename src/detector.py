#!/usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from belief_manager_msgs.srv import *


class Detector (object):

    def __init__(self):
        rospy.init_node('detector_node', anonymous=False)
        self.bridge = CvBridge()
        rospy.Subscriber("iris_0/usb_cam/image_raw", Image, self.image_callback)
        rospy.Subscriber("iris_0/usb_cam/camera_info", CameraInfo, self.info_callback)
        rospy.Subscriber("drone1/self_localization/pose", PoseStamped, self.pose_callback)
        self.addBelief1 = rospy.ServiceProxy("drone1/add_belief", AddBelief)
        self.removeBelief1 = rospy.ServiceProxy("drone1/remove_belief", RemoveBelief)
        self.queryBelief1 = rospy.ServiceProxy("drone1/query_belief", QueryBelief)
        self.addBelief2 = rospy.ServiceProxy("drone2/add_belief", AddBelief)
        self.removeBelief2 = rospy.ServiceProxy("drone2/remove_belief", RemoveBelief)
        self.queryBelief2 = rospy.ServiceProxy("drone2/query_belief", QueryBelief)
        #self.resp = self.addBelief1(belief_expression = "belief(1000, (1, 1, 1))", multivalued = True)
        #self.queryBelief1 = rospy.ServiceProxy("drone1/query_belief", QueryBelief)
        #resp = self.queryBelief1(query = "belief(1000, ?y)")
        #print (resp.substitutions)
        #print (resp.substitutions.split('\n')[0].split(':'))


        self._image_from_detector = rospy.Publisher('image_from_detector', Image, queue_size=30)
        self.distance = 2.0
        self.x = 0.0
        self.y = 0.0
        self.focal_length = 0.0
        self.id = 10
        self.positions = []
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Mask for green color
        low_green = np.array([25, 52, 72])
        high_green = np.array([102, 255, 255])
        green_mask = cv2.inRange(hsv_frame, low_green, high_green)
        green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

        # Edge detection
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY) #_INV 255
        kernal = np.ones((2, 2), np.uint8)
        dilation = cv2.dilate(thresh, kernal, iterations=2)
        cnt, hierarchy = cv2.findContours(
            dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rgb = cv2.cvtColor(green, cv2.COLOR_BGR2RGB)

        # Plot contours and text in image
        cv2.drawContours(cv_image, cnt, -1, (0, 0, 255), 2)
        text = "Number of plants in the image: " + str(len(cnt))
        cv2.putText(cv_image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (195, 193, 93), 2) 

        for i, c in enumerate(cnt):
            x,y,w,h = cv2.boundingRect(c)
            center = (x+w//2, y+h//2)
            if ((cv_image.shape[0]-70) > center[1] > 70 and (cv_image.shape[1]-73) > center[0] > 73):
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                n_pixels_x = (cv_image.shape[0]/2) - center[1]
                n_pixels_y = (cv_image.shape[1]/2) - center[0]
                h_length = (self.distance * w)/self.focal_length
                ratio = h_length/w
                desp_x = n_pixels_x * ratio
                desp_y = n_pixels_y * ratio
                position = Pose()
                position.position.x = str(round(self.x + desp_x, 2))
                position.position.y = str(round(self.y + desp_y, 2))
                position.position.z = str(round(0.0, 2))
                queryPos = "position(?x, (" + position.position.x + " ," + position.position.y + " ," + position.position.z + "))"
                #queryPos = "position(?x, ?y)"
                resp = self.queryBelief1(query = queryPos)
                
                if (str(resp.success) == "False"):     
                    print ("entra aqui")
                    out = True

                    for pos in self.positions:
                        if ((((float(position.position.x) + 0.2) > float(pos.position.x)) and (float(pos.position.x) > (float(position.position.x) - 0.2))) and (((float(position.position.y) + 0.2) > float(pos.position.y)) and (float(pos.position.y) > (float(position.position.y) - 0.2)))):
                            print("condicion se cumple")
                            out = False
                            break
                    if (out):
                        beliefPlant = "object(" + str(self.id) + ", plant)"
                        self.addBelief1(belief_expression = beliefPlant, multivalued = True)
                        self.addBelief2(belief_expression = beliefPlant, multivalued = True)
                        beliefName = "name(" + str(self.id) + ", plant" + str(self.id) + ")"
                        self.addBelief1(belief_expression = beliefName, multivalued = True)
                        self.addBelief2(belief_expression = beliefName, multivalued = True)
                        beliefPos = "position(" +  str(self.id) + ", (" + position.position.x + " ," + position.position.y + " ," + position.position.z + "))"
                        self.resp = self.addBelief1(belief_expression = beliefPos, multivalued = True)
                        self.addBelief2(belief_expression = beliefPos, multivalued = True)
                        print ("tamaño bounding box!!")
                        print (w)
                        if (30 < w < 100):
                            beliefFert = "unfertilized(" + str(self.id) + ")"
                            self.addBelief1(belief_expression = beliefFert, multivalued = True)
                            self.addBelief2(belief_expression = beliefFert, multivalued = True)
                        self.id = self.id + 1
                        self.positions.append(position)
                """else:
                    
                    queryP = "position(" + resp.substitutions.split('\n')[0].split(':')[1].replace(" ", "") + ", ?y)"
                    print (queryP)
                    resp = self.queryBelief1(query = queryP)
                    respPos = resp.substitutions.replace(" ", "").split(':')[1].replace("(", "").replace(")", "").split(",")
                    posX = float(respPos[0])
                    posY = float(respPos[1])
                    if ((position.position.x + 0.4) < posX or (position.position.y + 0.4) < posY):"""




        self._image_from_detector.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    
    def info_callback(self, msg):
        self.focal_length = msg.P[0]
    
    def pose_callback(self, msg):
        self.distance = msg.pose.position.z
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

if __name__ == '__main__':
    base = Detector()