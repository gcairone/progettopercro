#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

def main():
    rospy.init_node('camera_node', anonymous=True)
    
    image_pub = rospy.Publisher('/usb_camera/image_raw', Image, queue_size=10)

    bridge = CvBridge()

    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! image/jpeg ! jpegdec ! videoconvert ! appsink")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        rospy.logerr("Errore nell'aprire la fotocamera")
        return

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Errore nel ricevere il frame dalla fotocamera")
            break
        
        try:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione del frame: {e}")
            continue

        ros_image.header = Header()
        ros_image.header.stamp = rospy.Time.now()

        image_pub.publish(ros_image)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
