#!/usr/bin/python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import numpy as np



  
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
      
      objectPoints = np.array([(1, 1, 0),(1, 0, 0),(0,0,0), (0, 1, 0)],dtype=np.float32)
      imagePoints = np.array([ptA, ptB, ptC, ptD],dtype=np.float32)
      # CAmera Info in /camera_info
      cameraMatrix = np.array([(762.7249337622711, 0.0, 640.5), (0.0, 762.7249337622711, 360.5), (0.0, 0.0, 1.0)],dtype=np.float32)
      distCoeffs = np.zeros((4,1))
      
      criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
      corners2 = cv2.cornerSubPix(gray,imagePoints,(11,11),(-1,-1),criteria)
      success, rotation_vector, translation_vector = cv2.solvePnP(objectPoints,corners2, cameraMatrix, distCoeffs)
      
      # # project 3D points to image plane
      axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
      imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, cameraMatrix, distCoeffs)
      
 
      
      imagePoints = imagePoints.astype('int32')
      imgpts = imgpts.astype('int32')
      corners2 = corners2.astype('int32')
      image = self.draw(image,corners2,imgpts)
      
      
      # # draw the bounding box of the AprilTag detection
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
    
  def draw(self, img, corners, imgpts):
    corner = tuple(corners[2].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

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