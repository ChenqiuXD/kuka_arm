import cv2 
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from time import sleep

class imageProcess:
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.callback)
        sleep(1)
        self.sub.unregister()
        self.sub = rospy.Subscriber("camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        if hasattr(data, "K"):
            self.cameraMat = data.K
            print(self.cameraMat)
        else:
            print(data.step)

def main():
    rospy.init_node('image_process', anonymous=True)
    ic = imageProcess()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()