import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class SumNumsSub(Node):

    def __init__(self):
        super().__init__('sum_nums_publisher_node')
        self.subscription = self.create_subscription(
            Int8,
            'input_number',
            self.subscriber_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.num = 0
        self.sum = 0

    def subscriber_callback(self, msg):
        self.num = int(msg.data)
        self.sum = self.recursive_sum(self.num)
        print("Sum of first", self.num, "integer numbers is:", self.sum)

    def recursive_sum(self, n):
        if n <= 1:
            return n
        else:
            return n + self.recursive_sum(n-1)

def main(args=None):
    rclpy.init(args=args)

    sum_nums_subscriber_node = SumNumsSub()

    rclpy.spin(sum_nums_subscriber_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sum_nums_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()