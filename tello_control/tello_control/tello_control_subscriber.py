import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Classification
from cv_bridge import CvBridge
import cv2
from djitellopy import Tello
from collections import OrderedDict

class TelloControlSub(Node):
    """
    Create a TelloControlSub class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('tello_control_sub')

        # Initialize drone
        self.drone = Tello()
        self.drone.connect("")
        self.result_action = None
        self.last_action = None
        self.classification_threshold = 0.6
        self.frame = None
        self.publish = True
        
        # Create the image publisher
        self.image_publisher = self.create_publisher(Image,'/classification/input/image',1)
        self.timer_image_publish = self.create_timer(0.1,self.image_publish_callback)
        self.timer_image_publish.cancel()

        # Create the results subscriber
        self.subscription = self.create_subscription(Classification, '/classification/output/results', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()
    
    def listener_callback(self, msg):
        """
        Callback function.
        """
        results_dict = OrderedDict()
        # Convert vision_msgs.msg.ObjectHypothesis to python dictionary
        for result in msg.results:
            results_dict[result.class_id] = result.score
        # Sort the dictionary based on values (will bring the highest score item on top)
        results_dict = OrderedDict(sorted(results_dict.items(), key=lambda item: item[1], reverse=True))
        # Access the top result as drone action
        top_action = next(iter(results_dict))
        # Accept action if the classification score is above threshold 
        if results_dict[top_action] > self.classification_threshold:
            self.result_action = top_action
            self.command_drone(self.result_action)

    def command_drone(self, cmd):
        print("command rcvd:", cmd)
        # match cmd:
        #     case "takeoff":
        #         self.drone.takeoff()
        #         return
        #     case "land":
        #         self.drone.land()
        #         return
        #     case "forward":
        #         self.drone.move_forward()
        #         return
        #     case "back":
        #         self.drone.move_back()
        #         return
        #     case "left":
        #         self.drone.move_left()
        #         return
        #     case "right":
        #         self.drone.move_right()
        #         return
        #     case "noise":
        #         return
        pass

    def image_publish_callback(self):
        if self.publish:
            print("publishing drone image")
            self.drone.streamon()
            self.frame = self.drone.get_frame_read()
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.frame,"bgr8"))
            self.drone.streamoff()
        
        self.timer_image_publish.reset()
        pass

def main(args=None):
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    tello_control_subscriber_node = TelloControlSub()
    
    # Spin the node so the callback function is called.
    rclpy.spin(tello_control_subscriber_node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tello_control_subscriber_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()