from sum_nums_srv.srv import SumNumsSrv
import rclpy
from rclpy.node import Node


class SumNumsClient(Node):

    def __init__(self):
        super().__init__('sum_nums_client_node')
        self.cli = self.create_client(SumNumsSrv, 'sum_n_nums')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' SumNumsSrv service not available, waiting...')
        self.req = SumNumsSrv.Request()

    def send_request(self, n):
        self.req.n = n
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    sum_nums_client_node = SumNumsClient()

    num_input = int(input("Enter a positive integer number: "))
    if num_input < 0:
        print("Input number is not positive")
    else:
        print('Entered:', num_input)
        response = sum_nums_client_node.send_request(num_input)    
        print("Sum of first", num_input, "integer numbers is:", response.sum)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sum_nums_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()