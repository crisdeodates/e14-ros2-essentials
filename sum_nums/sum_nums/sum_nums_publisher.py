import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class SumNumsPub(Node):

    def __init__(self):
        super().__init__('sum_nums_publisher_node')
        self.publisher_ = self.create_publisher(Int8, 'input_number', 10)

    def sum_nums_publish(self, data):
        self.publisher_.publish(data)

def main(args=None):
    rclpy.init(args=args)

    sum_nums_publisher_node = SumNumsPub()
    num_data = Int8()

    num_input = int(input("Enter a positive integer number: "))
    if num_input < 0:
        print("Input number is not positive")
    else:
        print('Entered:', num_input)
        num_data.data = num_input
        sum_nums_publisher_node.sum_nums_publish(num_data)

    #rclpy.spin(sum_nums_publisher_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    sum_nums_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()