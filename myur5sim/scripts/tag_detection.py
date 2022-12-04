#!/usr/bin/python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import apriltag

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/myur5/camera1/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      print(e)
      rospy.logerr(e)
    
    image = cv_image
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    for r in results:
      # extract the bounding box (x, y)-coordinates for the AprilTag
      # and convert each of the (x, y)-coordinate pairs to integers
      (ptA, ptB, ptC, ptD) = r.corners
      ptB = (int(ptB[0]), int(ptB[1]))
      ptC = (int(ptC[0]), int(ptC[1]))
      ptD = (int(ptD[0]), int(ptD[1]))
      ptA = (int(ptA[0]), int(ptA[1]))
      # draw the bounding box of the AprilTag detection
      cv2.line(image, ptA, ptB, (0, 255, 0), 2)
      cv2.line(image, ptB, ptC, (0, 255, 0), 2)
      cv2.line(image, ptC, ptD, (0, 255, 0), 2)
      cv2.line(image, ptD, ptA, (0, 255, 0), 2)
      # draw the center (x, y)-coordinates of the AprilTag
      (cX, cY) = (int(r.center[0]), int(r.center[1]))
      cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
      # draw the tag family on the image
      tagFamily = r.tag_family.decode("utf-8")
      cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # show the output image after AprilTag detection
    cv2.imshow("Image", image)
    cv2.waitKey(1)

def main():
  a = camera_1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()