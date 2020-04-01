import cv2 
import rospy
from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco
import tf
import numpy as np
import threading

class imageProcess:
    def __init__(self):
        self.bridge = CvBridge()
        self.cameraMat = None
        self.rvec = None
        self.tvec = None
        self.corners = []
        self.ids = []
        
        self.pubTargetPose = rospy.Publisher("/kuka_arm/target_camera_cord", PoseStamped, queue_size=10)
        
        # First acquire the K matrix of the camera
        self.sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.paraCallback)
        sleep(1)
        self.sub.unregister()
        # Then start to process the image information
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imgCallback, queue_size=1)

    def paraCallback(self, data):
        # Get the camera intrinsic parameters
        self.cameraMat = np.array([[data.K[0], data.K[1], data.K[2]],
                            [data.K[3], data.K[4], data.K[5]],
                            [data.K[6], data.K[7], data.K[8]]])

    def imgCallback(self, data):
        # Use cvBridge to convert the sensor_msgs/Image data type to Mat data type
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.corners, self.ids, frame_marker = self.imgProcess(cv_image)
        self.calculateTransform()
        self.pubPose()
        # dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # frameAxis = aruco.drawAxis(frame_marker, self.cameraMat, dist, self.rvec, self.tvec, 0.1)
        # cv2.imshow("Image Window", frameAxis)
        cv2.imshow("Image Window", frame_marker)
        key = cv2.waitKey(2)
        if key==ord('q'):
            rospy.signal_shutdown("Keyboard pressed q")

    def imgProcess(self, frame):
        """
            This function detect the aruco tags in the frame
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        return corners, ids, frame_markers

    def calculateTransform(self):
        """
            This function calculate the transform of aruco tag respect to the camera coordinate
        """
        dist = np.array([0, 0, 0, 0, 0])  # The distortion matrix
        if self.corners:
            self.rvec, self.tvec, _ = aruco.estimatePoseSingleMarkers(self.corners, 0.08, self.cameraMat, dist)

    def pubPose(self):
        if self.rvec!=None:
            # Transform the tvec and rvec and publish the target object in camera coordinate
            R = np.array([[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,1]], dtype=float)
            R[:3, :3], _ = cv2.Rodrigues(self.rvec)
            quaternion = tf.transformations.quaternion_from_matrix(R)

            pose = PoseStamped()
            pose.header.frame_id = "camera_frame"
            pose.pose.position.x = self.tvec[0][0][0]
            pose.pose.position.y = self.tvec[0][0][1]
            pose.pose.position.z = self.tvec[0][0][2]
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            # print("Publishing pose")
            self.pubTargetPose.publish(pose)

def main():
    rospy.init_node('image_process', anonymous=True)
    ic = imageProcess()
    rospy.spin()


if __name__ == "__main__":
    main()