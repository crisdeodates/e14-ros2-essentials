import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import os

class FaceDetectorPub(Node):
    """
    Create a FaceDetectorPub class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('face_detector_pub')
        
        # Create the publisher. This publisher will publish an Image
        # to the video_frame_data topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frame_data', 10)
        
        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Load the haar cascade classifier
        self.haar_path = os.path.expanduser('~') + '/ros2_ws/src/face_detector/resource/haar_classifier.xml'
        self.face_cascade = cv2.CascadeClassifier(self.haar_path)
        
    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        # Convert to gray scale image
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ## Detect faces & returns positions of faces as Rect(x,y,w,h)
        face_rects = self.face_cascade.detectMultiScale(frame_gray, 1.3, 5)

        # Draw rectangles representing the detected faces
        for (x,y,w,h) in face_rects:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    def release_camera(self):
        self.cap.release()
    
def main(args=None):
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    face_detector_publisher_node = FaceDetectorPub()
    
    # Spin the node so the callback function is called.
    rclpy.spin(face_detector_publisher_node)

    face_detector_publisher_node.release_camera()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    face_detector_publisher_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()