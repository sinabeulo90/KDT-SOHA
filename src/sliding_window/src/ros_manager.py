import rospy

from xycar_motor.msg import xycar_motor
from subscribers.camera_subscriber import CameraSubscriber

class RosManager():
    def __init__(self, name="driver"):
        self.motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        self.camera_sub = CameraSubscriber()
        rospy.init_node("driver")
    
    
    def get_image(self):
        return self.camera_sub.get()


if __name__ == "__main__":
    import cv2 as cv

    controller = RosManager()
    while not rospy.is_shutdown():
        ret, img = controller.get_image()
        if ret:
            cv.imshow("test", img)
            cv.waitKey(10)
