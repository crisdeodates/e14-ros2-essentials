import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FaceDetectorSub(Node):
    """
    Create a FaceDetectorSub class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node.
        """
        # Initiate the Node class's constructor and give it a name.
        super().__init__('face_detector_sub')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'video_frame_data', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images.
        self.br = CvBridge()
        self.running = True
    
    def listener_callback(self, data):
        """
        Callback function to receive and display images.
        """    
        if self.running:
            # Convert ROS Image message to OpenCV image.
            current_frame = self.br.imgmsg_to_cv2(data)
            
            # Display image.
            cv2.imshow("face_detector_stream", current_frame)

            # Raise SystemExit exception to quit if ESC key is pressed.
            if cv2.waitKey(1) == 27:
                self.running = False
                cv2.destroyAllWindows()
                raise SystemExit

def main(args=None):
    """
    Main function to initialize the node and start the face detector subscriber.
    """
    # Initialize the rclpy library.
    rclpy.init(args=args)
    
    # Create the node.
    face_detector_subscriber_node = FaceDetectorSub()
    
    # Spin the node so the callback function is called.
    try:
        rclpy.spin(face_detector_subscriber_node)
    # Exit if SystemExit exception is raised.
    except SystemExit:
        print("Face detector stream output stopped")
    
    # Destroy the node explicitly.
    face_detector_subscriber_node.destroy_node()
    
    # Shutdown the ROS client library for Python.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
