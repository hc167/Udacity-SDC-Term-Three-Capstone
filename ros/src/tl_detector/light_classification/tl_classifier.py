from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def detect_state(self, image):
        pass

    def get_classification(self, images):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if images == None or len(images) == 0:
            return TrafficLight.UNKNOWN
        
        # For now, I just use the first light only. We may need to revisit this to check all the light images we detected.
        resized_image = cv2.resize(images[0], (30, 90)) 
        
        hsv_bb_img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
                
        red1 = cv2.inRange(hsv_bb_img, (0, 70, 50), (10, 255, 255)) 
        red2 = cv2.inRange(hsv_bb_img, (170, 70, 50), (180, 255, 255)) 

        yellow = cv2.inRange(hsv_bb_img, (40.0/360*255, 100, 100), (66.0/360*255, 255, 255)) 
        green = cv2.inRange(hsv_bb_img, (90.0/360*255, 100, 100), (140.0/360*255, 255, 255))

        a = cv2.countNonZero(red1) + cv2.countNonZero(red2)
        b = cv2.countNonZero(yellow)
        c = cv2.countNonZero(green)
        
        if a > 40:
            print('Red Light Detected!')
            return TrafficLight.RED
        elif b > 20 :
            print('Yellow Light Detected!')
            return TrafficLight.YELLOW
        elif c > 20 :
            print('Green Light Detected!')
            return TrafficLight.GREEN
        else:
            print ('Cannot determine color of light!')
            return TrafficLight.UNKNOWN
