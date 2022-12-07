#!/usr/bin/python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import numpy as np
import rotm2euler
import math
from std_msgs.msg import Float32MultiArray

from scipy.spatial.transform import Rotation 

    
class TagPoseEstimator:

  def __init__(self):
    self.pose_error_publisher = rospy.Publisher("/pose_error", Float32MultiArray, queue_size=1)
    self.image_sub = rospy.Subscriber("/myur5/camera1/image_raw", Image, self.callback)
    # Camera Info in /camera_info 
    self.cameraMatrix = np.array([(762.7249337622711, 0.0, 640.5), (0.0, 762.7249337622711, 360.5), (0.0, 0.0, 1.0)],dtype=np.float32)
    self.distCoeffs = np.zeros((4,1))
    self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # project 3D points to image plane
    self.axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
    self.tag_points = np.array([(1, 1, 0),(1, 0, 0),(0,0,0), (0, 1, 0)],dtype=np.float32)
    self.estimated_pose = np.identity(n=4, dtype=np.float64) # Estimated pose matrix
    
    
  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      print(e)
      rospy.logerr(e)
    
    image = cv_image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    apriltag_options = apriltag.DetectorOptions(families="tag36h11")
    apriltag_detector = apriltag.Detector(apriltag_options)
    results = apriltag_detector.detect(gray_image)
    
    for r in results:
      # extract the bounding box (x, y)-coordinates for the AprilTag
      # and convert each of the (x, y)-coordinate pairs to integers
      (ptA, ptB, ptC, ptD) = r.corners
      ptB = (int(ptB[0]), int(ptB[1]))
      ptC = (int(ptC[0]), int(ptC[1]))
      ptD = (int(ptD[0]), int(ptD[1]))
      ptA = (int(ptA[0]), int(ptA[1]))
      image_points = np.array([ptA, ptB, ptC, ptD],dtype=np.float32)
      refined_corners = cv2.cornerSubPix(gray_image,image_points,(11,11),(-1,-1),self.criteria)
      
      
      rotation_vector, translation_vector, points_projection = self.find_pose(refined_corners)
      
      image_points = image_points.astype('int32')
      points_projection = points_projection.astype('int32')
      refined_corners = refined_corners.astype('int32')
      
      image = self.draw_tag(image,r,ptA,ptB,ptC,ptD)
      image = self.draw_pose(image,refined_corners,points_projection)
      
      self.image = image
      self.origin = image_points[2]
      self.estimate_pose(rotation_vector,translation_vector)
      float_array = Float32MultiArray()
      float_array.data = [self.dx, self.dy, self.yaw]
      self.pose_error_publisher.publish(float_array)
      rate.sleep()
      self.draw_desired_pose()
      
      
      
      
      
      
    cv2.imshow("Image", image)
    cv2.waitKey(1)
    
    # When everything done, release the capture
    

  
  def draw_desired_pose(self):
    self.image = cv2.line(self.image, (int(self.image.shape[1]/2),int(self.image.shape[0]/2)), (int(self.image.shape[1]/2)+50,int(self.image.shape[0]/2)), (255,0,0), 5)
    self.image = cv2.line(self.image, (int(self.image.shape[1]/2),int(self.image.shape[0]/2)), (int(self.image.shape[1]/2),int(self.image.shape[0]/2)-50), (0,255,0), 5)


    
  def find_pose(self,corners):
    success, rotation_vector, translation_vector = cv2.solvePnP(self.tag_points,corners, self.cameraMatrix, self.distCoeffs)
    points_projection, jacobian = cv2.projectPoints(self.axis, rotation_vector, translation_vector, self.cameraMatrix, self.distCoeffs)
    return rotation_vector, translation_vector, points_projection
  
  def estimate_pose(self,rotation_vector,translation_vector):
    rvec_flipped = -rotation_vector.ravel()
    tvec_flipped = -translation_vector.ravel()
    
    # Convert rvec to a rotation matrix, and then to a Euler angles
    R, jacobian = cv2.Rodrigues(rvec_flipped)
    
    
    # From image plane to world coordinates
    realworld_tvec = np.dot(R, tvec_flipped)
    realworld_tvec[1], realworld_tvec[2] = -realworld_tvec[1], -realworld_tvec[2]
    
    # Conversion euler angles in radians, and then to degrees
    self.pitch, self.roll, self.yaw =  rotm2euler.rotation_matrix_to_euler_angles(R)
    pitch, roll, yaw = math.degrees(self.pitch), math.degrees(self.roll), math.degrees(self.yaw)
    estimated_euler_angles = np.array([roll, pitch, yaw])
    
    # Construct homogeneous transformation matrix
    self.estimated_pose[:3, :3] = R
    self.estimated_pose[:3, 3] = realworld_tvec
    

    self.target_homogeneous_matrix = np.array([(0,0,0,0),(0,0,0,0),(0,0,0,0),(0,0,0,1)])
    t = np.matmul(np.linalg.inv(self.estimated_pose), self.target_homogeneous_matrix)
    self.dx = t[0][3]
    self.dy = t[1][3]
    #self.alpha = math.atan2(self.dy, self.dx)

    # rotational_matrix = np.array([
    #                             [t[0][0], t[0][1], t[0][2]],
    #                             [t[1][0], t[1][1], t[1][2]],
    #                             [t[2][0], t[2][1], t[2][2]],
    #                         ])

    # r = Rotation.from_matrix(rotational_matrix)
    # self.beta = r.as_euler('XYZ', degrees=False)[2]
    
    dx = round(self.dx,2)
    dy = round(self.dy,2)
    yaw = round(self.yaw,2)
    text = f"dX: {str(dx)} dY:{str(dy)} yaw: {str(yaw)}"

    
    cv2.putText(self.image, text, (self.origin[0], self.origin[1] - 15),
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    
    
    
  def draw_tag(self,image,r,ptA,ptB,ptC,ptD):
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
    return image
    
  def draw_pose(self, img, corners, imgpts):
    corner = tuple(corners[2].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img


  
  
  
def main():
  a = TagPoseEstimator()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    rate = rospy.Rate(1000)
    main()