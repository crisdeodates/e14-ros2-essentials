from sum_nums_srv.srv import SumNumsSrv
import rclpy
from rclpy.node import Node


class SumNumsServer(Node):

    def __init__(self):
        super().__init__('sum_nums_server_node')
        self.srv = self.create_service(SumNumsSrv, 'sum_n_nums', self.service_callback)
        self.num = 0
        self.sum = 0
        print("Starting service sum_nums_srv")

    def service_callback(self, request, response):
        self.num = request.n
        self.sum = self.recursive_sum(self.num)
        response.sum = self.sum
        return response

    def recursive_sum(self, n):
        if n <= 1:
            return n
        else:
            return n + self.recursive_sum(n-1)


def main():
    rclpy.init()

    sum_nums_server_node = SumNumsServer()

    rclpy.spin(sum_nums_server_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()