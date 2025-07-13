from sum_nums_srv.srv import SumNumsSrv
import rclpy
from rclpy.node import Node


class SumNumsServer(Node):
    """
    Node to provide a service to compute the sum of first n integer numbers.
    """

    def __init__(self):
        """
        Initializes the SumNumsServer class.
        """
        super().__init__('sum_nums_server_node')
        self.srv = self.create_service(SumNumsSrv, 'sum_n_nums', self.service_callback)
        self.num = 0
        self.sum = 0
        print("Starting service sum_nums_srv")

    def service_callback(self, request, response):
        """
        Callback function to handle service requests and compute the sum of first n integer numbers.

        Parameters:
            request: The service request containing the number n.
            response: The service response to be filled with the computed sum.
        """
        self.num = request.n
        self.sum = self.recursive_sum(self.num)
        response.sum = self.sum
        return response

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
            return n + self.recursive_sum(n-1)


def main():
    """
    Main function to initialize the node and spin it to handle service requests.
    """
    rclpy.init()

    sum_nums_server_node = SumNumsServer()

    rclpy.spin(sum_nums_server_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
