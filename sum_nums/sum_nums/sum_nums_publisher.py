import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class SumNumsPub(Node):
    """
    Node to publish positive integer numbers.
    """
    def __init__(self):
        """
        Initializes the SumNumsPub class.
        """
        super().__init__('sum_nums_publisher_node')
        # Create a publisher for data type Int8
        self.publisher_ = self.create_publisher(Int8, 'input_number', 10)

    def sum_nums_publish(self, data):
        """
        Publishes the given data.

        Parameters:
            data: Data to be published.
        """
        self.publisher_.publish(data)

def main(args=None):
    """
    Main function to initialize the node and publish a positive integer number.
    """
    rclpy.init(args=args)
    sum_nums_publisher_node = SumNumsPub()
    num_data = Int8()

    # Accept a keyboard input
    num_input = int(input("Enter a positive integer number: "))
    if num_input < 0:
        print("Input number is not positive")
    else:
        print('Entered:', num_input)
        num_data.data = num_input
        # Publish the integer to the topic
        sum_nums_publisher_node.sum_nums_publish(num_data)

    # Destroy the node explicitly
    sum_nums_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
