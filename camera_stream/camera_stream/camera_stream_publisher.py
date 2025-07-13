import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStreamPub(Node):
    """
    Create a CameraStreamPub class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node.
        """
        # Initiate the Node class's constructor and give it a name.
        super().__init__('camera_stream_pub')
        
        # Create the publisher. This publisher will publish an Image
        # to the video_frame_data topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frame_data', 10)
        
        # We will publish a message every 0.1 seconds.
        timer_period = 0.1  # seconds
        
        # Create the timer.
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object.
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
            
        # Used to convert between ROS and OpenCV images.
        self.br = CvBridge()
        
    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame.
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message.
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
    
def main(args=None):
    """
    Main function to initialize the node and start the camera stream publisher.
    """
    # Initialize the rclpy library.
    rclpy.init(args=args)
    
    # Create the node.
    camera_stream_publisher_node = CameraStreamPub()
    
    # Spin the node so the callback function is called.
    rclpy.spin(camera_stream_publisher_node)
    
    # Destroy the node explicitly.
    camera_stream_publisher_node.destroy_node()
    
    # Shutdown the ROS client library for Python.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
