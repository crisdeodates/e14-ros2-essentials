import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class SumNumsSub(Node):
    """
    Node to subscribe to a topic and compute the sum of first n integer numbers.
    """

    def __init__(self):
        """
        Initializes the SumNumsSub class.
        """
        super().__init__('sum_nums_publisher_node')
        # Create a subscription to the 'input_number' topic
        self.subscription = self.create_subscription(
            Int8,
            'input_number',
            self.subscriber_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.num = 0
        self.sum = 0

    def subscriber_callback(self, msg):
        """
        Callback function to compute the sum of first n integer numbers.

        Parameters:
            msg: The received message containing the number n.
        """
        self.num = int(msg.data)
        self.sum = self.recursive_sum(self.num)
        print("Sum of first", self.num, "integer numbers is:", self.sum)

    def recursive_sum(self, n):
        """
        Recursive function to compute the sum of first n integer numbers.

        Parameters:
            n: The number of integer numbers to sum.

        Returns:
            The sum of the first n integer numbers.
        """
        if n <= 1:
            return n
        else:
            return n + self.recursive_sum(n - 1)


def main(args=None):
    """
    Main function to initialize the node and spin it to handle messages.
    """
    rclpy.init(args=args)

    sum_nums_subscriber_node = SumNumsSub()

    rclpy.spin(sum_nums_subscriber_node)
    
    # Destroy the node explicitly
    sum_nums_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
